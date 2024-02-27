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
#include <SDL2/SDL_opengl.h>

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
#include "../imgui-1.90/backends/imgui_impl_opengl3.h"
#include "../imgui-1.90/backends/imgui_impl_sdl2.h"
#include "../imgui-1.90/imgui.h"
#include "../imgui-knobs-main/imgui-knobs.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
SDL_GLContext usad_gl_context_;
SDL_Window* usad_window_;
SDL_WindowFlags usad_window_flags_;

constexpr unsigned int display_hist_size_ = 128;
constexpr int leane_default_tdc_ = 2048;

ImFont* font_default;
ImFont* font_dseg_big;
ImFont* font_dseg;

class UsadGUI : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ira_interfaces::msg::EncodersTicks>::SharedPtr
        encoders_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leane_abs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    bool show_encoders_window_ = true;
    bool show_speedometer_window_ = true;
    bool show_cmd_vel_window_ = true;

    float skf_l_ratio_mpt_, skf_r_ratio_mpt_;

    ira_interfaces::msg::EncodersTicks encoders_ticks_latest_;
    float left_wheel_ticks_hist_[display_hist_size_] = {0};
    float right_wheel_ticks_hist_[display_hist_size_] = {0};
    rcl_time_point_value_t encoders_dt_ns_;

    std_msgs::msg::Int32 leane_abs_latest_;
    float leane_abs_hist_[display_hist_size_] = {0};
    int leane_tdc_ = leane_default_tdc_;

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
            static rcl_time_point_value_t prev_ts_ns = 0;
            auto now_ns = this->now().nanoseconds();
            this->encoders_dt_ns_ = now_ns - prev_ts_ns;
            prev_ts_ns = now_ns;
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

        // Update all values in background

        // Process events
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

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(usad_window_);
        ImGui::NewFrame();

        // Draw main window
        {
            ImGui::SetNextWindowSize(ImVec2(320, 210));
            ImGui::Begin("Toolbox", nullptr,
                         ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_AlwaysAutoResize);

            ImGui::Checkbox("Encoders", &show_encoders_window_);
            if (this->show_encoders_window_) {
                this->draw_encoders_window(&this->show_encoders_window_);
            }

            ImGui::Checkbox("Speedometer", &show_speedometer_window_);
            if (this->show_speedometer_window_) {
                this->draw_speedometer_window(&this->show_speedometer_window_);
            }

            ImGui::Checkbox("Velocity Command", &show_speedometer_window_);
            if (this->show_cmd_vel_window_) {
                this->draw_cmd_vel_window(&this->show_cmd_vel_window_);
            }

            ImGui::Separator();
            ImGui::Text("Rendering: %.3f ms/frame (%.1f FPS)",
                        1000.0f / ImGui::GetIO().Framerate,
                        ImGui::GetIO().Framerate);
            if (ImGui::CollapsingHeader("About")) {
                ImGui::TextWrapped(
                    "USAD GUI\n"
                    "Copyright (C) 2022  Jacopo Maltagliati\n"
                    "Released under the Apache v2 License.\n\n"
                    "Dear ImGui v1.90\n"
                    "Copyright (c) 2014-2023 Omar Cornut\n"
                    "Released under the MIT license.\n\n"
                    "ImGui Knobs\n"
                    "Copyright (c) 2022 Simon Altschuler\n"
                    "Released under the MIT license.\n\n"
                    "DSEG Fonts 0.46\n"
                    "Copyright (c) 2020 Keshikan (www.keshikan.net)\n"
                    "Licensed under SIL OPEN FONT LICENSE Version 1.1.\n\n"
                    "Roboto Font\n"
                    "Copyright (c) 2015 Google Inc.\n"
                    "Released under the Apache v2 License.\n\n");
                ImGui::Separator();
                ImGui::Text("Built on ImGui v%s (%d)", IMGUI_VERSION,
                            IMGUI_VERSION_NUM);
            }
        }
        ImGui::End();

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x,
                   (int)ImGui::GetIO().DisplaySize.y);
        glClearColor(clear_color_.x * clear_color_.w,
                     clear_color_.y * clear_color_.w,
                     clear_color_.z * clear_color_.w, clear_color_.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(usad_window_);
    }

    void draw_encoders_window(bool* visible) {
        // ImGui::SetNextWindowSize(ImVec2(640, 480), ImGuiCond_Once);
        ImGui::Begin("Encoders", visible, ImGuiWindowFlags_AlwaysAutoResize);
        // SKF
        ImGui::BeginGroup();
        int latest_l = (int)this->encoders_ticks_latest_.left_wheel_ticks;
        int latest_r = (int)this->encoders_ticks_latest_.right_wheel_ticks;
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Text("SKF Left Wheel (ticks)");
        ImGui::SliderInt("##", &latest_l, 0, 100);
        ImGui::PlotLines("##", this->left_wheel_ticks_hist_,
                         IM_ARRAYSIZE(this->left_wheel_ticks_hist_), 0, nullptr,
                         -100.f, 100.f, ImVec2(0, 80.f));
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        ImGui::BeginGroup();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Text("SKF Right Wheel (ticks)");
        ImGui::SliderInt("##", &latest_r, 0, 100);
        ImGui::PlotLines("##", this->right_wheel_ticks_hist_,
                         IM_ARRAYSIZE(this->right_wheel_ticks_hist_), 0,
                         nullptr, -100.f, 100.f, ImVec2(0, 80.f));
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        // Leane
        int latest_lea = this->leane_abs_latest_.data;
        float latest_lea_percent =
            (this->leane_abs_latest_.data - this->leane_tdc_) / 40.96f;
        ImGui::BeginGroup();
        ImGuiKnobs::Knob("Leane Orientation", &latest_lea_percent, -50.f, 50.f,
                         0.1f, "%.1f%%", ImGuiKnobVariant_Tick, 125.f);
        ImGui::BeginGroup();
        ImGui::Text("TDC");
        ImGui::SameLine();
        if (ImGui::Button("Set")) {
            this->leane_tdc_ = latest_lea;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            this->leane_tdc_ = leane_default_tdc_;
        }
        ImGui::EndGroup();
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::Text("Leane Data (raw)");
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::SliderInt("##", &latest_lea, 0, 4096);
        ImGui::PlotLines("##", this->leane_abs_hist_,
                         IM_ARRAYSIZE(this->leane_abs_hist_), 0, nullptr, 0.f,
                         4096.f, ImVec2(0, 100.f));
        ImGui::EndGroup();
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
            trip_odo += distance / 1000.f;
            speed_kph = distance / (dt_ns / 1000000000.f) * 3.6f;
            if (speed_kph > max_speed_kph) max_speed_kph = speed_kph;
        }
        // if (speed_kph > max_speed_kph) max_speed_kph = speed_kph;
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(15, 135, 250, 255));
        ImGui::PushFont(font_dseg_big);
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
        ImGui::PushFont(font_dseg);
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
        ImGui::Text("Top Speed: %2.0fkm/h - Trip: %3.2fkm", max_speed_kph,
                    trip_odo);
        ImGui::End();
    }

    void draw_cmd_vel_window(bool* visible) {
        ImGui::Begin("Velocity Command", visible,
                     ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text(
            "Speed Setpoint: %.3fm/s - Angle Setpoint: %.3frad - Total "
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
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) !=
        0) {
        fprintf(stderr, "Error: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    usad_window_flags_ =
        (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                          SDL_WINDOW_ALLOW_HIGHDPI);
    usad_window_ =
        SDL_CreateWindow("USAD GUI", SDL_WINDOWPOS_CENTERED,
                         SDL_WINDOWPOS_CENTERED, 1280, 720, usad_window_flags_);
    usad_gl_context_ = SDL_GL_CreateContext(usad_window_);
    SDL_GL_MakeCurrent(usad_window_, usad_gl_context_);
    SDL_GL_SetSwapInterval(1);  // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // (void)io;
#if 0
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
        // Enable Gamepad Controls
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        // IF using Docking Branch
#endif
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(usad_window_, usad_gl_context_);
    ImGui_ImplOpenGL3_Init();

    font_default = io.Fonts->AddFontFromFileTTF(
        "default.ttf", 16.0f, NULL, io.Fonts->GetGlyphRangesDefault());
    font_dseg_big = io.Fonts->AddFontFromFileTTF(
        "dseg.ttf", 192.0f, NULL, io.Fonts->GetGlyphRangesDefault());
    font_dseg = io.Fonts->AddFontFromFileTTF("dseg.ttf", 48.0f, NULL,
                                             io.Fonts->GetGlyphRangesDefault());

    // RCL
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsadGUI>());
    rclcpp::shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(usad_gl_context_);
    SDL_DestroyWindow(usad_window_);
    SDL_Quit();
    exit(EXIT_SUCCESS);
}
