/**
 * @file usad_gui.cpp
 * @author Jacopo Maltagliati
 * @brief Graphical User Interface for the USAD Vehicle.
 *
 * @copyright Copyright (c) 2023 Jacopo Maltagliati.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <SDL2/SDL.h>

#include <chrono>
#include <mutex>
#include <queue>
#include <string>

// ROS
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ira_interfaces/msg/encoders_ticks.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/int32.hpp"

// ImGui
#include "../imgui-1.90/backends/imgui_impl_sdl2.h"
#include "../imgui-1.90/backends/imgui_impl_sdlrenderer2.h"
#include "../imgui-1.90/imgui.h"
#include "../imgui-knobs-main/imgui-knobs.h"

#ifndef FONTS_PREFIX
#error The FONTS_PREFIX variable is not set. Check your CMakeLists.txt!
#else
#define XSTR(s) STR(s)
#define STR(s) #s
#define FONT_LOC(fname) XSTR(FONTS_PREFIX) "/" fname
#endif

#if !SDL_VERSION_ATLEAST(2, 0, 17)
#error This application requires SDL 2.0.17 or above.
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

SDL_GLContext usad_gl_context_;
SDL_Renderer* usad_renderer_;
SDL_Window* usad_window_;
SDL_WindowFlags usad_window_flags_;

constexpr ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImFont* font_default_;
ImFont* font_dseg_;
ImFont* font_dseg_big_;

constexpr unsigned int display_hist_size_ = 128;
constexpr int leane_top_ = 4096;
constexpr int leane_bottom_ = 0;
constexpr int leane_max_real_ = 3550;
constexpr int leane_min_real_ = 900;

class UsadGUI : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ira_interfaces::msg::EncodersTicks>::SharedPtr
        encoders_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leane_abs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    bool show_encoders_window_ = false;
    bool show_speedometer_window_ = false;
    bool show_cmd_vel_window_ = false;

    float skf_l_ratio_mpt_, skf_r_ratio_mpt_;

    ira_interfaces::msg::EncodersTicks encoders_ticks_latest_;
    float left_wheel_ticks_hist_[display_hist_size_] = {0};
    float right_wheel_ticks_hist_[display_hist_size_] = {0};
    rcl_time_point_value_t encoders_prev_ts_ns_ = 0;
    rcl_time_point_value_t encoders_dt_ns_;

    std_msgs::msg::Int32 leane_abs_latest_;
    float leane_abs_hist_[display_hist_size_] = {0};
    int leane_tdc_ = (leane_top_ + leane_bottom_) / 2;
    int leane_min_ = leane_top_;
    int leane_max_ = leane_bottom_;

    float cmd_vel_x_, cmd_vel_theta_;
    unsigned int cmd_vel_count_;

    void leane_abs_callback(const std_msgs::msg::Int32 msg) {
        static uint offset = 0;
        this->leane_abs_latest_ = msg;
        this->leane_abs_hist_[offset] = msg.data;
        offset++;
        if (offset >= display_hist_size_) offset = 0;
    }

    void encoders_ticks_callback(const ira_interfaces::msg::EncodersTicks msg) {
        static uint offset = 0;
        {
            auto now_ns = this->now().nanoseconds();
            this->encoders_dt_ns_ = now_ns - this->encoders_prev_ts_ns_;
            this->encoders_prev_ts_ns_ = now_ns;
        }
        this->encoders_ticks_latest_ = msg;
        this->left_wheel_ticks_hist_[offset] = msg.left_wheel_ticks;
        this->right_wheel_ticks_hist_[offset] = msg.right_wheel_ticks;
        offset++;
        if (offset >= display_hist_size_) offset = 0;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
        this->cmd_vel_x_ = cmd_vel->linear.x;
        this->cmd_vel_theta_ = cmd_vel->angular.z;
        this->cmd_vel_count_++;
    }

    void gui_callback() {
        static bool done = false;
        if (done) rclcpp::shutdown();

        // Process Events
        {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                ImGui_ImplSDL2_ProcessEvent(&event);
                if (event.type == SDL_QUIT) done = true;
                if (event.type == SDL_WINDOWEVENT &&
                    event.window.event == SDL_WINDOWEVENT_CLOSE &&
                    event.window.windowID == SDL_GetWindowID(usad_window_))
                    done = true;
            }
        }

        // Start a new Frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame(usad_window_);
        ImGui::NewFrame();

        // Draw the UI
        {
            ImGui::SetNextWindowSize(ImVec2(350, 210));
            ImGui::Begin(
                "Toolbox", nullptr,
                ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);

            ImGui::Checkbox("Encoders", &show_encoders_window_);
            if (this->show_encoders_window_) {
                this->draw_encoders_window(&this->show_encoders_window_);
            }

            ImGui::Checkbox("Speedometer", &show_speedometer_window_);
            if (this->show_speedometer_window_) {
                this->draw_speedometer_window(&this->show_speedometer_window_);
            }

            ImGui::Checkbox("Velocity Command", &show_cmd_vel_window_);
            if (this->show_cmd_vel_window_) {
                this->draw_cmd_vel_window(&this->show_cmd_vel_window_);
            }

            ImGui::Separator();

            // Draw Control Board Status and DeltaT
            {
                static float disp_packet_dt_ms = 0;
                float last_packet_dt_ms =
                    (this->now().nanoseconds() - this->encoders_prev_ts_ns_) /
                    (float)1e6;
                if (last_packet_dt_ms < 0 || last_packet_dt_ms > 60000.f) {
                    disp_packet_dt_ms = -1;
                } else if (last_packet_dt_ms > 45.f) {
                    disp_packet_dt_ms = last_packet_dt_ms;
                }
                ImGui::Text("Packet DT: %.3f ms", disp_packet_dt_ms);
                ImGui::Text("Control Board Status: ");
                ImGui::SameLine();
                if (last_packet_dt_ms > 100.f) {
                    ImGui::TextColored({1.f, 0.f, 0.f, 1.f}, "OFFLINE");
                } else if (last_packet_dt_ms > 55.f) {
                    ImGui::TextColored({1.f, 0.55f, 0.f, 1.f}, "ONLINE (HL)");
                } else {
                    ImGui::TextColored({0.f, 1.f, 0.f, 1.f}, "ONLINE");
                }
            }

            if (ImGui::CollapsingHeader("About")) {
                ImGui::TextWrapped(
                    "USAD GUI\n"
                    "Copyright (C) 2022  Jacopo Maltagliati\n"
                    "Released under the Apache v2 License.\n"
                    "\n"
                    "Dear ImGui v1.90\n"
                    "Copyright (c) 2014-2023 Omar Cornut\n"
                    "Released under the MIT license.\n"
                    "\n"
                    "ImGui Knobs\n"
                    "Copyright (c) 2022 Simon Altschuler\n"
                    "Released under the MIT license.\n"
                    "\n"
                    "DSEG Fonts 0.46\n"
                    "Copyright (c) 2020 Keshikan (www.keshikan.net)\n"
                    "Licensed under SIL OPEN FONT LICENSE Version 1.1.\n"
                    "\n"
                    "Roboto Font\n"
                    "Copyright (c) 2015 Google Inc.\n"
                    "Released under the Apache v2 License.\n");
                ImGui::Separator();
                ImGui::Text("Render time: %.3f ms/frame (%.1f FPS)",
                            1000.0f / ImGui::GetIO().Framerate,
                            ImGui::GetIO().Framerate);
                ImGui::Text("Built on ImGui v%s (%d)", IMGUI_VERSION,
                            IMGUI_VERSION_NUM);
            }
        }

        ImGui::End();

        // Render the Frame
        {
            ImGui::Render();
            SDL_RenderSetScale(usad_renderer_,
                               (int)ImGui::GetIO().DisplayFramebufferScale.x,
                               (int)ImGui::GetIO().DisplayFramebufferScale.y);
            SDL_SetRenderDrawColor(
                usad_renderer_, (Uint8)(clear_color_.x * 255),
                (Uint8)(clear_color_.y * 255), (Uint8)(clear_color_.z * 255),
                (Uint8)(clear_color_.w * 255));
            SDL_RenderClear(usad_renderer_);
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
            SDL_RenderPresent(usad_renderer_);
        }
    }

    void draw_encoders_window(bool* visible) {
        // ImGui::SetNextWindowSize(ImVec2(640, 480), ImGuiCond_Once);
        ImGui::Begin("Encoders", visible, ImGuiWindowFlags_AlwaysAutoResize);
        // SKF
        int latest_l = (int)this->encoders_ticks_latest_.left_wheel_ticks;
        int latest_r = (int)this->encoders_ticks_latest_.right_wheel_ticks;
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::SeparatorText("SKF Wheel Bearing Encoders");
        ImGui::BeginGroup();
        ImGui::Text("Left Wheel (ticks)");
        ImGui::SliderInt("##", &latest_l, 0, 100);
        ImGui::PlotLines("##", this->left_wheel_ticks_hist_,
                         IM_ARRAYSIZE(this->left_wheel_ticks_hist_), 0, nullptr,
                         -100.f, 100.f, ImVec2(0, 80.f));
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        ImGui::BeginGroup();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Text("Right Wheel (ticks)");
        ImGui::SliderInt("##", &latest_r, 0, 100);
        ImGui::PlotLines("##", this->right_wheel_ticks_hist_,
                         IM_ARRAYSIZE(this->right_wheel_ticks_hist_), 0,
                         nullptr, -100.f, 100.f, ImVec2(0, 80.f));
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        // Leane
        ImGui::SeparatorText("Leane");
        int latest_lea = this->leane_abs_latest_.data;
        float latest_lea_percent =
            (this->leane_abs_latest_.data - this->leane_tdc_) / 40.96f;
        static bool auto_tdc = true;
        if (latest_lea < this->leane_min_ && latest_lea > leane_min_real_)
            this->leane_min_ = latest_lea;
        if (latest_lea > this->leane_max_ && latest_lea < leane_max_real_)
            this->leane_max_ = latest_lea;
        if (auto_tdc)
            this->leane_tdc_ = (this->leane_min_ + this->leane_max_) / 2;
        ImGui::BeginGroup();
        ImGuiKnobs::Knob("Orientation", &latest_lea_percent, -50.f, 50.f, 0.1f,
                         "%.1f%%", ImGuiKnobVariant_Tick, 125.f);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::Text("Data (raw)");
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::SliderInt("##", &latest_lea, 0, 4096);
        ImGui::PlotLines("##", this->leane_abs_hist_,
                         IM_ARRAYSIZE(this->leane_abs_hist_), 0, nullptr, 0.f,
                         4096.f, ImVec2(0, 100.f));
        ImGui::Checkbox("Auto TDC", &auto_tdc);
        ImGui::EndGroup();
        if (ImGui::CollapsingHeader("Advanced")) {
            ImGui::BeginGroup();
            if (!auto_tdc) {
                if (ImGui::Button("Set")) this->leane_tdc_ = latest_lea;
                ImGui::SameLine();
            }
            if (ImGui::Button("Reset")) {
                this->leane_tdc_ = (leane_top_ + leane_bottom_) / 2;
                this->leane_min_ = leane_top_;
                this->leane_max_ = leane_bottom_;
            }
            ImGui::SameLine();
            ImGui::Text("TDC: %d", this->leane_tdc_);
            ImGui::EndGroup();
            ImGui::Text("Min: %d - Max: %d - Mid: %d", this->leane_min_,
                        this->leane_max_,
                        (this->leane_max_ + this->leane_min_) / 2);
        }
        ImGui::End();
    }

    void draw_speedometer_window(bool* visible) {
        float speed_kph;
        float distance;
        static float max_speed_kph = .0f;
        static float trip_odo = .0f;
        ImGui::Begin("Speedometer", visible, ImGuiWindowFlags_AlwaysAutoResize);
        {
            int ticks_l, ticks_r;
            rcl_time_point_value_t dt_ns;
            ticks_l = (int)this->encoders_ticks_latest_.left_wheel_ticks;
            ticks_r = (int)this->encoders_ticks_latest_.right_wheel_ticks;
            dt_ns = this->encoders_dt_ns_;
            distance = (ticks_l * this->skf_l_ratio_mpt_ +
                        ticks_r * this->skf_r_ratio_mpt_) /
                       2;
            trip_odo += abs(distance / 1000.f);
            speed_kph = distance / (dt_ns / 1000000000.f) * 3.6f;
            if ((speed_kph > -1.f && speed_kph < 0.f) || isnan(speed_kph))
                speed_kph = .0f;
            if (speed_kph > max_speed_kph) max_speed_kph = speed_kph;
        }
        // if (speed_kph > max_speed_kph) max_speed_kph = speed_kph;
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(15, 135, 250, 255));
        ImGui::PushFont(font_dseg_big_);
        if (speed_kph < .0f) {
            speed_kph = fabs(speed_kph);
            if (speed_kph < 10.f) {
                ImGui::Text("!%1.0f.", speed_kph);
            } else {
                ImGui::Text("%2.0f.", speed_kph);
            }
        } else {
            if (speed_kph < 10.f) {
                ImGui::Text("!%1.0f", speed_kph);
            } else {
                ImGui::Text("%2.0f", speed_kph);
            }
        }
        ImGui::PopFont();
        ImGui::PushFont(font_dseg_);
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGuiKnobs::Knob("##", &speed_kph, 0.f, 50.f, 0.1f, "",
                         ImGuiKnobVariant_WiperOnly, 144.f,
                         ImGuiKnobFlags_NoTitle | ImGuiKnobFlags_NoInput);
        ImGui::Text("KM/H");
        ImGui::PopStyleColor();
        ImGui::EndGroup();
        ImGui::PopFont();
        ImGui::Separator();
        if (ImGui::Button("Reset")) {
            max_speed_kph = .0f;
            trip_odo = .0f;
        }
        ImGui::SameLine();
        ImGui::Text("Top Speed: %2.0f km/h - Trip: %3.2f km", max_speed_kph,
                    trip_odo);
        ImGui::End();
    }

    void draw_cmd_vel_window(bool* visible) {
        ImGui::Begin("Velocity Command", visible,
                     ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text(
            "Speed Setpoint: %.3f m/s\nAngle Setpoint: %.3f rad\nTotal "
            "Commands: %d",
            this->cmd_vel_x_, this->cmd_vel_theta_, this->cmd_vel_count_);
        ImGui::End();
    }

   public:
    UsadGUI() : Node("usad_gui") {
        this->declare_parameter("skf_l_ratio_mpt", 0.007225956f);
        this->declare_parameter("skf_r_ratio_mpt", 0.007178236f);
        this->skf_l_ratio_mpt_ =
            (float)this->get_parameter("skf_l_ratio_mpt").as_double();
        this->skf_r_ratio_mpt_ =
            (float)this->get_parameter("skf_r_ratio_mpt").as_double();
        this->encoders_ticks_sub_ =
            this->create_subscription<ira_interfaces::msg::EncodersTicks>(
                "encoders_ticks", 10,
                std::bind(&UsadGUI::encoders_ticks_callback, this, _1));
        this->leane_abs_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "leane_abs", 10, std::bind(&UsadGUI::leane_abs_callback, this, _1));
        this->cmd_vel_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&UsadGUI::cmd_vel_callback, this, _1));
        this->timer_ = this->create_wall_timer(
            32ms, std::bind(&UsadGUI::gui_callback, this));
    }
};

int main(int argc, char** argv) {
    // SDL Initialization
    {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
            std::stringstream ss;
            ss << "Unable to initialize SDL. Reason: " << SDL_GetError();
            throw std::runtime_error(ss.str().c_str());
        }

#ifdef SDL_HINT_IME_SHOW_UI
        SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

        usad_window_flags_ =
            (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        usad_window_ = SDL_CreateWindow("USAD GUI", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, 1280, 720,
                                        usad_window_flags_);
        if (usad_window_ == nullptr) {
            std::stringstream ss;
            ss << "Unable to create SDL Window. Reason: " << SDL_GetError();
            throw std::runtime_error(ss.str().c_str());
        }

        usad_renderer_ = SDL_CreateRenderer(
            usad_window_, -1,
            SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
        if (usad_renderer_ == nullptr) {
            std::stringstream ss;
            ss << "Unable to create SDL Renderer. Reason: " << SDL_GetError();
            throw std::runtime_error(ss.str().c_str());
        }
    }

    // ImGui Initialization
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = NULL;

        font_default_ = io.Fonts->AddFontFromFileTTF(
            FONT_LOC("Roboto-Regular.ttf"), 20.0f, NULL,
            io.Fonts->GetGlyphRangesDefault());
        font_dseg_big_ = io.Fonts->AddFontFromFileTTF(
            FONT_LOC("DSEG14Classic-Italic.ttf"), 192.0f, NULL,
            io.Fonts->GetGlyphRangesDefault());
        font_dseg_ = io.Fonts->AddFontFromFileTTF(
            FONT_LOC("DSEG14Classic-Italic.ttf"), 48.0f, NULL,
            io.Fonts->GetGlyphRangesDefault());

#ifdef USE_LIGHT_THEME
        ImGui::StyleColorsLight();
#endif

        ImGui_ImplSDL2_InitForSDLRenderer(usad_window_, usad_renderer_);
        ImGui_ImplSDLRenderer2_Init(usad_renderer_);
    }

    // RCL
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsadGUI>());
    rclcpp::shutdown();

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(usad_gl_context_);
    SDL_DestroyWindow(usad_window_);
    SDL_Quit();
    exit(EXIT_SUCCESS);
}
