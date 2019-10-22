#pragma once

#include <chrono>
#include <cstddef>
#include <list>
#include <string>
#include <vector>

#include <terminalpp/ansi_terminal.hpp>
#include <terminalpp/terminalpp.hpp>

#include "ProgressReporter.h"

struct UIState
{
  ProgressReporter& get_progress_reporter();

  const std::list<std::string>& get_messages() const;
  void push_message(std::string message);

  constexpr static size_t MAX_MESSAGES = 6;

private:
  ProgressReporter _progress_reporter;

  std::list<std::string> _messages;
};

struct TerminalRenderBuffer
{

  const terminalpp::element& at(uint32_t x, uint32_t y) const;

  std::vector<terminalpp::element> elements;
  uint32_t width;
  uint32_t height;
};

enum class TerminalLayoutType
{
  FixedWidth,
  Flexible
};

struct TerminalUIElement
{
  virtual ~TerminalUIElement();

  virtual TerminalRenderBuffer render() const = 0;

  TerminalLayoutType get_layout_type() const;

protected:
  TerminalLayoutType _layout_type;
};

struct TerminalLabel : TerminalUIElement
{
  TerminalLabel();
  virtual ~TerminalLabel();

  virtual TerminalRenderBuffer render() const override;

  const std::string& get_content() const;
  const terminalpp::colour& get_color() const;

  void set_content(const std::string& content);
  void set_color(const terminalpp::colour& color);

private:
  std::string _content;
  terminalpp::colour _color;
};

struct TerminalMultilineLabel : TerminalLabel
{
  TerminalMultilineLabel();
  virtual ~TerminalMultilineLabel();

  virtual TerminalRenderBuffer render() const override;

  void set_line_width(uint32_t line_width);

private:
  uint32_t _line_width;
};

struct TerminalProgressBar : TerminalUIElement
{
  TerminalProgressBar();
  virtual ~TerminalProgressBar();

  TerminalRenderBuffer render() const override;

  float get_progress() const;
  void set_progress(float progress);

  void set_allowed_width(uint32_t allowed_width);

private:
  float _progress;
  uint32_t _allowed_width;
};

struct TerminalUI
{
  explicit TerminalUI(UIState* state);

  void redraw();

private:
  constexpr static std::chrono::milliseconds TERMINAL_REDRAW_INTERVAL =
    std::chrono::milliseconds{ 50 };
  std::chrono::high_resolution_clock::time_point _last_redraw_time;

  UIState* _state;

  terminalpp::ansi_terminal _terminal;
  terminalpp::screen _screen;
  terminalpp::canvas _canvas;

  TerminalLabel _header_label;

  TerminalLabel _loading_label;
  TerminalProgressBar _loading_progress_bar;
  TerminalLabel _loading_progress_label;

  TerminalLabel _indexing_label;
  TerminalProgressBar _indexing_progress_bar;
  TerminalLabel _indexing_progress_label;

  // TerminalLabel _postprocessing_label;
  // TerminalProgressBar _postprocessing_progress_bar;
  // TerminalLabel _postprocessing_progress_label;

  TerminalLabel _log_label;
};