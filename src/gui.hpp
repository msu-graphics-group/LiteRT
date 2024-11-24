#ifndef NURBS_SRC_GUI
#define NURBS_SRC_GUI

#include <string>
#include <functional>
#include <vector>

#include <SDL.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <Image2d.h>
#include <LiteMath.h>
#include <cinttypes>

#include "raytracer.hpp"

struct Application
{
public:
  Application(const std::string &name, int width, int height);
public:
  void set_context();
public:
  void fill_buffer(const LiteImage::Image2D<uint32_t> &src);
  void start_new_frame();
  void draw();
public:
  void poll_events();
  void add_sdl_callback(std::function<void(SDL_Event)> callback);
  void add_imgui_callback(std::function<void(void)> callback);
public:
  SDL_Window *get_window() const { return window_; }
public:
  ~Application();
private:
  SDL_Window *window_;
  SDL_Renderer *renderer_;
  SDL_Texture *texture_;
  SDL_Rect scr_rect_;
  ImGuiContext *context_;
  std::vector<std::function<void(SDL_Event)>> sdl_callbacks_;
  std::vector<std::function<void(void)>> imgui_callbacks_;
public:
  void default_resize_callback(SDL_Event event);
};

void setup_default_style(ImGuiStyle &style);

#endif
