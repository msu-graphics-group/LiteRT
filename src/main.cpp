#include <iostream>
#include <filesystem>
#include <cmath>
#include <cinttypes>
#include <string_view>
#include <cstdio>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <SDL2/SDL.h>
#include <Image2d.h>
#include <LiteMath.h>

#include "Surface.hpp"
#include "raytracer.hpp"
#include "utils.hpp"

using namespace LiteMath;

void setup_imgui_style();
ImFont* load_imgui_font(ImGuiIO &io);
void copy_image_to_texture(
    const LiteImage::Image2D<uint32_t> &image,
    SDL_Texture *texture);

// Main code
int main(int, char** argv)
{
  std::filesystem::path exec_path = std::filesystem::canonical(argv[0]);
  std::filesystem::current_path(exec_path.parent_path().parent_path());

  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) ;
  SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  atexit(SDL_Quit);

  int WIDTH = 1200;
  int HEIGHT = 800;
  float aspect = static_cast<float>(WIDTH)/HEIGHT;
  float fov = M_PI_4;

  // Create window with SDL_Renderer graphics context
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE);
  SDL_Window* window = SDL_CreateWindow("Basic NURBS Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, window_flags);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC|SDL_RENDERER_ACCELERATED);
  SDL_RendererInfo info;


  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

  // Setup Dear ImGui style
  setup_imgui_style();
  // New font
  ImFont *font = load_imgui_font(io);

  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  bool to_load_surface = false;
  bool surface_changed = false;
  bool renderer_changed = false;
  bool shading_changed = false;
  std::string default_path_to_surf = std::filesystem::current_path() / "resources";
  char path_to_surf[10000] = {};
  std::copy(default_path_to_surf.begin(), default_path_to_surf.end(), path_to_surf);

  std::vector<RBezierGrid> rbeziers;
  std::vector<std::vector<BoundingBox3d>> bboxes;
  std::vector<std::vector<LiteMath::float2>> uvs;
  FrameBuffer fb = { 
    LiteImage::Image2D<uint32_t>(WIDTH, HEIGHT, LiteMath::uchar4{ 153, 153, 153, 255 }.u32),
    LiteImage::Image2D<float>(WIDTH, HEIGHT, std::numeric_limits<float>::infinity())
  };
  SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
  SDL_Rect scr_rect = { 0, 0, WIDTH, HEIGHT };

  Camera camera(aspect, fov, {}, {});

  // Main loop
  bool done = false;

  bool camera_move = false;
  bool camera_changed = false;
  int prev_x = 0, prev_y = 0;

  
  const char *renderers[] = { 
    "Regular Sample Points", 
    "Newton Method (in progress)",
    "Bounding boxes"
  };

  const char *shaders[] = {
    "UV",
    "Normals"
  };
  std::function<ShadeFuncType> shader_funcs[] = {
    shade_uv,
    shade_normals
  };

  int cur_renderer = 0;
  int cur_shader = 0;
  float ms = 0.0f;

  while (!done)
  {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        done = true;
      if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && 
          event.window.windowID == SDL_GetWindowID(window))
        done = true;
      if (event.type = SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
        SDL_GetWindowSize(window, &WIDTH, &HEIGHT);
        camera = Camera(WIDTH*1.0f/HEIGHT, camera.fov, camera.position, camera.target);
        camera_changed = true;
        fb.col_buf.resize(WIDTH, HEIGHT);
        fb.z_buf.resize(WIDTH, HEIGHT);
        SDL_DestroyTexture(texture);
        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
        scr_rect.w = WIDTH;
        scr_rect.h = HEIGHT;
      }
    }

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
      if (!camera_move) {
        camera_move = true;
        SDL_GetMouseState(&prev_x, &prev_y);
      } else {
        int x, y;
        SDL_GetMouseState(&x, &y);
        int dx = x - prev_x;
        int dy = y - prev_y;
        prev_x = x;
        prev_y = y;
        float distance = length(camera.position-camera.target);
        float3 new_pos  = camera.position 
                        + camera.up * dy * distance / 150.0f
                        - camera.right * dx * distance / 150.0f;
        camera.position = camera.target + normalize(new_pos-camera.target)*distance;
        camera = Camera(camera.aspect, camera.fov, camera.position, camera.target);
      }
    } 
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
      camera_move = false;
    }

    if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
      SDL_Delay(10);
      continue;
    }

    if (camera_move || camera_changed || surface_changed || renderer_changed || shading_changed) {
      fb.col_buf.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
      fb.z_buf.clear(std::numeric_limits<float>::infinity());
    }
    surface_changed = false;
    camera_changed = false;

    //Render image
    auto b = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < rbeziers.size(); ++i) {
      switch(cur_renderer)
      {
        case 0: draw_points(rbeziers[i], camera, fb, 250/std::sqrt(rbeziers.size()), shader_funcs[cur_shader]); break;
        case 1: draw_newton(rbeziers[i], camera, fb, shader_funcs[cur_shader]); break;
        case 2: draw_boxes(bboxes[i], uvs[i], camera, fb);
      }
    }
    auto e = std::chrono::high_resolution_clock::now();
    ms = std::chrono::duration_cast<std::chrono::microseconds>(e-b).count()/1000.0f;

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();        

    ImGui::PushFont(font);
    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
    {  
      ImGui::SetNextWindowPos({0, 0});
      ImGui::SetNextWindowSize({ 333*1.0f, HEIGHT*1.0f });
      ImGui::Begin("Preferences", nullptr, 
          ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize);
      ImGui::Text("Surface preferences:");
      if (ImGui::Button("Load new surface")) {
        to_load_surface = true;
      }
      //ImGui::ColorPicker3("Surface Color", surf_color);
      ImGui::Text("Camera settings:");
      camera_changed |= ImGui::DragFloat3("Camera position", camera.position.M);
      camera_changed |= ImGui::DragFloat3("Camera target", camera.target.M);
      camera = Camera(camera.aspect, camera.fov, camera.position, camera.target);
      ImGui::Text("Renderer settings:");
      renderer_changed = 
          ImGui::ListBox("Method", &cur_renderer, renderers, sizeof(renderers)/sizeof(*renderers));
      shading_changed =
          ImGui::ListBox("Shading", &cur_shader, shaders, sizeof(shaders)/sizeof(*shaders));
      ImGui::Text("Debug Info");
      ImGui::Text("\tApplication average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
      ImGui::Text("\tCurrent render time: %.3f ms/frame (%.1f FPS)", ms, 1000.0f/ms);
      ImGui::End();
    }

    if (to_load_surface) {
      FILE *f = popen("zenity --file-selection", "r");
      [[maybe_unused]] auto _ = fgets(path_to_surf, sizeof(path_to_surf), f);
      fclose(f);
      path_to_surf[strlen(path_to_surf)-1] = '\0';
      to_load_surface = false;
      surface_changed = true;
      try {
        rbeziers = load_rbeziers(path_to_surf);
        bboxes.resize(0);
        uvs.resize(0);
        for (auto &surf: rbeziers) {
          auto [cur_bboxes, cur_uv] = get_bvh_leaves(surf);
          bboxes.push_back(cur_bboxes);
          uvs.push_back(cur_uv);
        }
        BoundingBox3d bbox;
        for (auto &surf: rbeziers) {
          bbox.mn = LiteMath::min(bbox.mn, surf.bbox.mn);
          bbox.mx = LiteMath::max(bbox.mx, surf.bbox.mx);
        }
        auto target = bbox.center();
        float radius = LiteMath::length(bbox.mn-target);
        float distance = radius / std::sin(M_PI/8);
        camera = Camera(aspect, fov, { target.x, target.y, target.z+distance }, target);
        std::cout << "\"" << path_to_surf << "\" loaded!" << std::endl;
      } catch(...) {
        std::cout << "Failed to load \"" << path_to_surf << "\"!" << std::endl;
      }
    }
    ImGui::PopFont();

    // Rendering
    ImGui::Render();

    copy_image_to_texture(fb.col_buf, texture);
    SDL_RenderClear(renderer);
    SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
    SDL_RenderCopy(renderer, texture, &scr_rect, &scr_rect);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
  }
  
  auto save_path = std::filesystem::current_path().append("result.bmp");
  LiteImage::SaveBMP(save_path.c_str(), fb.col_buf.data(), WIDTH, HEIGHT);

  // Cleanup
  SDL_DestroyTexture(texture);
  io.Fonts->ClearFonts();
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);

  return 0;
}

void setup_imgui_style() {
  ImGuiStyle &style = ImGui::GetStyle();
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_TitleBg] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_Border] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
  style.Colors[ImGuiCol_FrameBg] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
  style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
  style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
  style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
  style.WindowRounding = 8.0f;
  style.ScrollbarRounding = 10.0f;
  style.FrameRounding = 5.0f;
  style.GrabRounding = 10.0f;
  style.GrabMinSize = 20.0f;
}

ImFont* load_imgui_font(ImGuiIO &io) {
  io.Fonts->AddFontDefault();
  auto path = std::filesystem::current_path().append("resources").append("Roboto-Black.ttf");
  return io.Fonts->AddFontFromFileTTF(path.c_str(), 14, nullptr, io.Fonts->GetGlyphRangesCyrillic());
}

void copy_image_to_texture(
    const LiteImage::Image2D<uint32_t> &image,
    SDL_Texture *texture) {
  char *pixels = nullptr;
  int pitch = 0;
  SDL_Rect scr_rect = { 0, 0, static_cast<int>(image.width()), static_cast<int>(image.height()) };
  SDL_LockTexture(texture, &scr_rect, reinterpret_cast<void**>(&pixels), &pitch);
  for (uint32_t y = 0; y < image.height(); ++y) {
    std::copy(
        image.data()+y*image.width(),
        image.data()+y*image.width()+image.width(),
        reinterpret_cast<uint32_t*>(pixels+y*pitch));
  }
  SDL_UnlockTexture(texture);
}
