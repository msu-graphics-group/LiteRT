#include <iostream>
#include <filesystem>
#include <cmath>
#include <cinttypes>
#include <string_view>
#include <cstdio>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#ifdef __APPLE__
    #include <SDL.h>
#else
    #include <SDL2/SDL.h>
#endif

#include <Image2d.h>
#include <LiteMath.h>

#include "Surface.hpp"
#include "raytracer.hpp"
#include "utils.hpp"
#include "embree_adaptors.hpp"
#include "gui.hpp"

using namespace LiteMath;
using namespace LiteImage;

int main(int, char** argv)
{
  std::filesystem::path exec_path = std::filesystem::canonical(argv[0]);
  std::filesystem::current_path(exec_path.parent_path().parent_path());
  auto proj_path = std::filesystem::current_path();

  int WIDTH = 1200;
  int HEIGHT = 800;
  float aspect = static_cast<float>(WIDTH)/HEIGHT;
  float fov = M_PI_4;

  Application app("Basic NURBS Viewer", WIDTH, HEIGHT);
  Camera camera(aspect, fov, {}, {});
  
  app.set_context();
  auto *font = ImGui::GetIO().Fonts->AddFontFromFileTTF(
    (proj_path / "resources" / "Roboto-Black.ttf").c_str(), 14);
  
  bool done = false;
  bool to_load_surface = false;
  bool surface_changed = false;
  bool renderer_changed = false;
  bool shading_changed = false;
  bool camera_changed = false;

  const char *renderers[] = { 
    "Regular Sample Points", 
    "Newton Method",
    "BVH Leaves",
    "Embree User Defined",
    "Embree Triangles"
  };
  int cur_renderer = 0;

  const char *shaders[] = {
    "UV",
    "Normals"
  };
  std::function<ShadeFuncType> shader_funcs[] = {
    shade_uv,
    shade_normals
  };
  int cur_shader = 0;

  embree::EmbreeScene embree_scn, embree_tesselated, embree_boxes;

  embree::RayPackSize ray_pack_sizes[] = { 
    embree::RayPackSize::RAY_PACK_1,
    embree::RayPackSize::RAY_PACK_4,
    embree::RayPackSize::RAY_PACK_8,
    embree::RayPackSize::RAY_PACK_16
  };
  int cur_ray_pack = 0;
  const char *ray_packs_str[] = {
    "x1", "x4", "x8", "x16"
  };

  std::vector<RBezierGrid> rbeziers;
  std::vector<std::vector<LiteMath::float2>> uvs;
  std::vector<std::vector<BoundingBox3d>> bboxes;
  int total_bboxes_count = 0;

  FrameBuffer fb = { 
    Image2D<uint32_t>(WIDTH, HEIGHT, uchar4{ 153, 153, 153, 255 }.u32),
    Image2D<float>(WIDTH, HEIGHT, std::numeric_limits<float>::infinity())
  };

  auto close_event_f = [&](SDL_Event event) {
    if (event.type == SDL_QUIT)
        done = true;
    if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && 
        event.window.windowID == SDL_GetWindowID(app.get_window()))
      done = true;
  };

  auto resize_event_f = [&](SDL_Event event) {
    if (event.type == SDL_EventType::SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED 
        && event.window.windowID == SDL_GetWindowID(app.get_window())) {
      SDL_GetWindowSize(app.get_window(), &WIDTH, &HEIGHT);
      fb.col_buf.resize(WIDTH, HEIGHT);
      fb.z_buf.resize(WIDTH, HEIGHT);
      camera = Camera(WIDTH*1.0f/HEIGHT, camera.fov, camera.position, camera.target);
      camera_changed = true;
    }
  };

  auto mouse_callback_f = [&, mouse_move = false, prev_x = 0, prev_y = 0]() mutable {
    auto &io = ImGui::GetIO();
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
      camera_changed = true;
      if (!mouse_move) {
        mouse_move = true;
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
      mouse_move = false;
    }
  };

  auto scroll_calback_f = [&](SDL_Event event) mutable {
    auto &io = ImGui::GetIO();
    if (event.type == SDL_EventType::SDL_MOUSEWHEEL
        && event.window.windowID == SDL_GetWindowID(app.get_window())
        && !io.WantCaptureMouse) {
      float r = event.wheel.y;
      float3 new_pos  = camera.position + normalize(camera.target-camera.position) * r;
      camera = Camera(camera.aspect, camera.fov, new_pos, camera.target);
      camera_changed = true;
    }
  };

  app.add_sdl_callback(close_event_f);
  app.add_sdl_callback(resize_event_f);
  app.add_sdl_callback(scroll_calback_f);
  app.add_imgui_callback(mouse_callback_f);

  app.set_context();
  while (!done)
  {
    app.poll_events();

    if (camera_changed || surface_changed || renderer_changed || shading_changed) {
      fb.col_buf.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
      fb.z_buf.clear(std::numeric_limits<float>::infinity());
    }
    surface_changed = false;
    camera_changed = false;

    //Render image
    auto b = std::chrono::high_resolution_clock::now();
    if (cur_renderer <= 1) {
      for (int i = 0; i < rbeziers.size(); ++i) {
        if (!rbeziers[i].is_visible)
          continue;
        switch(cur_renderer)
        {
          case 0: draw_points(rbeziers[i], camera, fb, 250/std::sqrt(rbeziers.size()), shader_funcs[cur_shader]); break;
          case 1: draw_newton(rbeziers[i], camera, fb, shader_funcs[cur_shader]); break;
        }
      }
    } else {
      switch(cur_renderer)
      {
        case 2: embree_boxes.draw(camera, fb, shader_funcs[cur_shader]); break;
        case 3: embree_scn.draw(camera, fb, shader_funcs[cur_shader], ray_pack_sizes[cur_ray_pack]); break;
        case 4: embree_tesselated.draw_triangles(camera, fb, shader_funcs[cur_shader]); break;
      }
    }
    auto e = std::chrono::high_resolution_clock::now();
    float ms = std::chrono::duration_cast<std::chrono::microseconds>(e-b).count()/1000.0f;      

    app.start_new_frame();
    ImGui::PushFont(font);
    //Preferences window
    {  
      ImGui::SetNextWindowPos({0, 0});
      ImGui::SetNextWindowSize({ 333*1.0f, HEIGHT*1.0f });
      ImGui::Begin("Preferences", nullptr, 
          ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize);
      ImGui::Text("Surface preferences:");
      if (ImGui::Button("Load new surface")) {
        to_load_surface = true;
      }
      ImGui::Text("Camera settings:");
      camera_changed |= ImGui::DragFloat3("Camera position", camera.position.M);
      camera_changed |= ImGui::DragFloat3("Camera target", camera.target.M);
      camera = Camera(camera.aspect, camera.fov, camera.position, camera.target);
      ImGui::Text("Renderer settings:");
      renderer_changed |= 
          ImGui::ListBox("Ray Pack Size", &cur_ray_pack, ray_packs_str, 4);
      ImGui::InputInt("Max Steps", &max_steps);
      ImGui::InputFloat("Intersection EPS", &EPS, 0.005f);
      renderer_changed |= 
          ImGui::ListBox("Method", &cur_renderer, renderers, sizeof(renderers)/sizeof(*renderers));
      shading_changed =
          ImGui::ListBox("Shading", &cur_shader, shaders, sizeof(shaders)/sizeof(*shaders));
      ImGui::Text("Debug Info");
      ImGui::Text("\tResolution: %dx%d", WIDTH, HEIGHT);
      ImGui::Text("\tSurfaces count: %lu", rbeziers.size());
      ImGui::Text("\tTotal boxes count: %d", total_bboxes_count);
      auto &io = ImGui::GetIO();
      ImGui::Text("\tApplication average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
      ImGui::Text("\tCurrent render time: %.3f ms/frame (%.1f FPS)", ms, 1000.0f/ms);
      ImGui::End();
    }

    //Inspector window
    {
      ImGui::SetNextWindowPos({WIDTH-200.0f, 0});
      ImGui::SetNextWindowSize({ 200*1.0f, HEIGHT*1.0f });
      ImGui::Begin("Inspector", nullptr, 
          ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize);
      for (int i = 0; i < rbeziers.size(); ++i) {
        std::string label = "Entity #" + std::to_string(i);
        surface_changed |= ImGui::Checkbox(label.c_str(), &rbeziers[i].is_visible);
      }
      ImGui::End();
    }
    ImGui::PopFont();


    //loading new surface
    if (to_load_surface) {
      embree_scn.clear_scene();
      embree_tesselated.clear_scene();
      embree_boxes.clear_scene();

      char path_to_surf[1000] = {};
      #ifndef USE_ZENITY
        std::cout << "Path to surface: ";
        std::cin >> path_to_surf;
      #else
        FILE *f = popen("zenity --file-selection", "r");
        [[maybe_unused]] auto _ = fgets(path_to_surf, sizeof(path_to_surf), f);
        pclose(f);
        path_to_surf[strnlen(path_to_surf, sizeof(path_to_surf))-1] = '\0';
      #endif
      to_load_surface = false;
      surface_changed = true;
      try {
        rbeziers = load_rbeziers(path_to_surf);
        bboxes.resize(0);
        uvs.resize(0);
        total_bboxes_count = 0;
        for (auto &surf: rbeziers) {
          auto [cur_bboxes, cur_uv] = get_bvh_leaves(surf);
          bboxes.push_back(cur_bboxes);
          uvs.push_back(cur_uv);
          total_bboxes_count += cur_bboxes.size();
        }
        for (int i = 0; i < rbeziers.size(); ++i) {
          embree_scn.attach_surface(rbeziers[i], bboxes[i], uvs[i]);
          embree_boxes.attach_boxes(bboxes[i], uvs[i]);
          embree_tesselated.attach_mesh(get_nurbs_control_mesh(rbeziers[i]));
        }
        embree_scn.commit_scene();
        embree_boxes.commit_scene();
        embree_tesselated.commit_scene();
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

    // Rendering
    app.fill_buffer(fb.col_buf);
    app.draw();
  }  
  auto save_path = std::filesystem::current_path().append("result.bmp");
  LiteImage::SaveBMP(save_path.c_str(), fb.col_buf.data(), WIDTH, HEIGHT);

  return 0;
}
