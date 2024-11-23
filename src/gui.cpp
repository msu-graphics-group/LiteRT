#include "gui.hpp"

static auto sdl_initialization = []{
  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) ;
  SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  atexit(SDL_Quit);
  return 0;
}();

Application::Application(const std::string &name, int width, int height) {
  // Create window with SDL_Renderer graphics context
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE);
  window_ = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, window_flags);
  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_PRESENTVSYNC|SDL_RENDERER_ACCELERATED);
  texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, width, height);
  scr_rect_ = { 0, 0, width, height };
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  context_ = ImGui::CreateContext();
  ImGui::SetCurrentContext(context_);
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window_, renderer_);
  ImGui_ImplSDLRenderer2_Init(renderer_);
  // set default style
  setup_default_style(ImGui::GetStyle());
  // add default callbacks
  add_sdl_callback([this](SDL_Event event){ this->default_resize_callback(event); });
  // add default font 
  ImGui::GetIO().Fonts->AddFontDefault();
}

void Application::add_imgui_callback(std::function<void(void)> callback) {
  imgui_callbacks_.emplace_back(std::move(callback));
}

void Application::add_sdl_callback(std::function<void(SDL_Event)> callback) {
  sdl_callbacks_.emplace_back(std::move(callback));
}

void Application::fill_buffer(const LiteImage::Image2D<uint32_t> &img) {
  char *pixels = nullptr;
  int pitch = 0;
  SDL_Rect scr_rect = { 0, 0, static_cast<int>(img.width()), static_cast<int>(img.height()) };
  SDL_LockTexture(texture_, &scr_rect, reinterpret_cast<void**>(&pixels), &pitch);
  for (uint32_t y = 0; y < img.height(); ++y) {
    std::copy(
        img.data()+y*img.width(),
        img.data()+(y+1)*img.width(),
        reinterpret_cast<uint32_t*>(pixels+y*pitch));
  }
  SDL_UnlockTexture(texture_);
}

void Application::draw() {
  set_context();
  auto &io = ImGui::GetIO();
  ImGui::Render();
  SDL_RenderClear(renderer_);
  SDL_RenderSetScale(renderer_, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
  SDL_RenderCopy(renderer_, texture_, &scr_rect_, &scr_rect_);
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer_);
  SDL_RenderPresent(renderer_);
}

void Application::default_resize_callback(SDL_Event event) {
  if (event.type == SDL_EventType::SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED 
      && event.window.windowID == SDL_GetWindowID(window_)) {
    SDL_GetWindowSize(window_, &scr_rect_.w, &scr_rect_.h);
    int WIDTH = scr_rect_.w;
    int HEIGHT = scr_rect_.h;
    SDL_DestroyTexture(texture_);
    texture_ = SDL_CreateTexture(
        renderer_, SDL_PIXELFORMAT_ABGR8888, 
        SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
  }
}

void Application::poll_events() {
  SDL_Event event;
  ImGui::SetCurrentContext(context_);
  while(SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    for (auto &sdl_callback: sdl_callbacks_)
      sdl_callback(event);
  }
  for (auto &imgui_callback: imgui_callbacks_)
    imgui_callback();
}

void Application::set_context() {
  ImGui::SetCurrentContext(context_);
}

Application::~Application() {
  // Cleanup
  ImGui::SetCurrentContext(context_);
  SDL_DestroyTexture(texture_);
  ImGui::GetIO().Fonts->ClearFonts();
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyRenderer(renderer_);
  SDL_DestroyWindow(window_);
}

void setup_default_style(ImGuiStyle &style) {
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

void Application::start_new_frame() {
  ImGui::SetCurrentContext(context_);
  // Start the Dear ImGui frame
  ImGui_ImplSDLRenderer2_NewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();  
}
