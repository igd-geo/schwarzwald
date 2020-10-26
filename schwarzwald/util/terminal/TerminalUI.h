#pragma once

#include <chrono>
#include <cstddef>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "debug/ProgressReporter.h"

struct UIState {
  ProgressReporter &get_progress_reporter();

  constexpr static size_t MAX_MESSAGES = 6;

private:
  ProgressReporter _progress_reporter;
};

enum class TerminalLayoutType { FixedWidth, Flexible };

struct TerminalUIElement {
  virtual ~TerminalUIElement();

  virtual void render(std::ostream &stream) const = 0;

  TerminalLayoutType get_layout_type() const;

protected:
  TerminalLayoutType _layout_type;
};

struct TerminalLabel : TerminalUIElement {
  TerminalLabel();
  virtual ~TerminalLabel();

  virtual void render(std::ostream &stream) const override;

  const std::string &get_content() const;
  const std::string &get_color() const;

  void set_content(const std::string &content);
  void set_color(const std::string &color);

private:
  std::string _content;
  std::string _color;
};

struct TerminalMultilineLabel : TerminalLabel {
  TerminalMultilineLabel();
  virtual ~TerminalMultilineLabel();

  virtual void render(std::ostream &stream) const override;

  void set_line_width(uint32_t line_width);

private:
  uint32_t _line_width;
};

struct TerminalProgressBar : TerminalUIElement {
  TerminalProgressBar();
  virtual ~TerminalProgressBar();

  void render(std::ostream &stream) const override;

  float get_progress() const;
  void set_progress(float progress);

  void set_allowed_width(uint32_t allowed_width);

private:
  float _progress;
  uint32_t _allowed_width;
};

struct TerminalUI {
  explicit TerminalUI(UIState *state);
  ~TerminalUI();

  void redraw();

private:
  constexpr static std::chrono::milliseconds TERMINAL_REDRAW_INTERVAL_WITH_TTY =
      std::chrono::milliseconds{50};
  constexpr static std::chrono::milliseconds
      TERMINAL_REDRAW_INTERVAL_WITHOUT_TTY = std::chrono::seconds{5};

  std::chrono::milliseconds _redraw_interval;
  std::chrono::high_resolution_clock::time_point _t_start;
  std::chrono::high_resolution_clock::time_point _last_redraw_time;

  void redraw_with_tty() const;
  void redraw_without_tty() const;

  void rebuild_progress_ui_with_tty();
  void rebuild_progress_ui_without_tty();

  UIState *_state;

  std::once_flag _init_draw_buffer_once;

  std::vector<std::vector<std::unique_ptr<TerminalUIElement>>> _ui_elements;
};

struct TerminalUIAsyncRenderer {
  explicit TerminalUIAsyncRenderer(TerminalUI &ui);
  ~TerminalUIAsyncRenderer();

  TerminalUIAsyncRenderer(const TerminalUIAsyncRenderer &) = delete;
  TerminalUIAsyncRenderer(TerminalUIAsyncRenderer &&) = delete;

  TerminalUIAsyncRenderer &operator=(const TerminalUIAsyncRenderer &) = delete;
  TerminalUIAsyncRenderer &operator=(TerminalUIAsyncRenderer &&) = delete;

private:
  TerminalUI &_ui;
  std::thread _render_thread;
  std::atomic_bool _do_render;
};