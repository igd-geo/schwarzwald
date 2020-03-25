#include "terminal/TerminalUI.h"

#include "terminal/stdout_helper.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace std::chrono;

const static std::string ForegroundColorWhite = "\u001b[37m";

static std::string pretty_print_large_number(double large_number) {
  constexpr std::array<const char *, 5> suffixes = {"", "K", "M", "B", "T"};

  const auto order_of_magnitude = [=](double val) {
    double cur_val = val;
    for (size_t order = 0; order < 4; ++order) {
      if (cur_val < 1000)
        return order;
      cur_val = std::round(cur_val / 1000);
    }
    return size_t{4};
  }(large_number);

  const auto suffix = suffixes[order_of_magnitude];
  const auto number_trimmed = large_number / std::pow(1000, order_of_magnitude);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << number_trimmed << suffix;
  return ss.str();
}

static std::string
format_progress_counter(const ProgressCounter_t &progress_counter) {
  std::stringstream ss;
  std::visit(
      [&ss](const auto &counter) {
        const auto cur_progress = counter.get_current_progress();
        const auto max_progress = counter.get_max_progress();
        const auto progress_percentage =
            100 * (static_cast<double>(cur_progress) / max_progress);

        ss << " " << std::fixed << std::setprecision(2) << progress_percentage
           << "% [" << pretty_print_large_number(cur_progress) << "/"
           << pretty_print_large_number(max_progress) << "]";
      },
      progress_counter);
  return ss.str();
}

/**
 * Returns a nice glyph for the given progress value in [0;1]
 */
static std::string get_progress_glyph(float progress) {
  constexpr static std::array<const char *, 9> Glyphs = {
      "\u2588", "\u2589", "\u258A", "\u258B", "\u258C",
      "\u258D", "\u258E", "\u258F", " "};
  const auto step =
      std::max(0, std::min(8, static_cast<int>((1.f - progress) * 8)));
  return Glyphs[step];
}

/**
 * Format a progress name (e.g. 'loading') into a right-justified format
 * with a colon at the end (e.g. 'loading    :')
 */
static std::string format_progress_name(const std::string &name,
                                        size_t max_name_length) {
  std::stringstream ss;
  ss << name;
  for (size_t fill = 0; fill < (max_name_length - name.size()); ++fill) {
    ss << ' ';
  }
  ss << ':';
  return ss.str();
}

ProgressReporter &UIState::get_progress_reporter() {
  return _progress_reporter;
}

TerminalUIElement::~TerminalUIElement() {}

TerminalLayoutType TerminalUIElement::get_layout_type() const {
  return _layout_type;
}

TerminalLabel::TerminalLabel() {
  _layout_type = TerminalLayoutType::FixedWidth;
}

TerminalLabel::~TerminalLabel() {}

void TerminalLabel::render(std::ostream &stream) const {
  stream << _color << _content;
}

const std::string &TerminalLabel::get_content() const { return _content; }

const std::string &TerminalLabel::get_color() const { return _color; }

void TerminalLabel::set_content(const std::string &content) {
  _content = content;
}

void TerminalLabel::set_color(const std::string &color) { _color = color; }

TerminalMultilineLabel::TerminalMultilineLabel() : _line_width(80) {}

TerminalMultilineLabel::~TerminalMultilineLabel() {}

void TerminalMultilineLabel::render(std::ostream &stream) const {

  const auto chunk_string =
      [](const std::string &str,
         uint32_t chunk_length) -> std::vector<std::string> {
    const auto num_chunks = static_cast<size_t>(
        std::ceil(str.size() / static_cast<double>(chunk_length)));
    std::vector<std::string> chunks;
    chunks.reserve(num_chunks);
    for (size_t chunk = 0; chunk < num_chunks; ++chunk) {
      const auto start_iter = str.begin() + (chunk * chunk_length);
      const auto end_iter = ((chunk + 1) * chunk_length >= str.size())
                                ? str.end()
                                : str.begin() + ((chunk + 1) * chunk_length);
      chunks.emplace_back(start_iter, end_iter);
    }

    return chunks;
  };

  const auto lines = chunk_string(get_content(), _line_width);

  for (auto &line : lines) {
    stream << get_color() << line << "\n";
  }
}

void TerminalMultilineLabel::set_line_width(uint32_t line_width) {
  _line_width = line_width;
}

TerminalProgressBar::TerminalProgressBar() : _progress(0), _allowed_width(0) {
  _layout_type = TerminalLayoutType::Flexible;
}

TerminalProgressBar::~TerminalProgressBar() {}

void TerminalProgressBar::render(std::ostream &stream) const {
  if (_allowed_width < 3)
    return;

  const auto steps = _allowed_width - 2;
  const auto enabled_steps = steps * _progress;

  // TODO Colors

  stream << '[';
  for (uint32_t idx = 0; idx < steps; ++idx) {
    const auto glyph =
        get_progress_glyph(enabled_steps - static_cast<float>(idx));
    stream << glyph;
  }
  stream << ']';
}

float TerminalProgressBar::get_progress() const { return _progress; }

void TerminalProgressBar::set_progress(float progress) { _progress = progress; }

void TerminalProgressBar::set_allowed_width(uint32_t allowed_width) {
  _allowed_width = allowed_width;
}

TerminalUI::TerminalUI(UIState *state) : _state(state) {}

void TerminalUI::redraw() {
  const auto time_since_last_redraw = duration_cast<milliseconds>(
      high_resolution_clock::now() - _last_redraw_time);
  if (time_since_last_redraw < TERMINAL_REDRAW_INTERVAL)
    return;
  _last_redraw_time = high_resolution_clock::now();

  // TODO Don't rebuild the UI completely, instead just update what has changed
  rebuild_progress_ui();

  if (_ui_elements.empty())
    return;

  // Redraw

  util::print_lock().lock();

  for (auto &ui_line : _ui_elements) {
    std::cout << "\u001b[2K";
    for (auto &ui_element : ui_line) {
      ui_element->render(std::cout);
    }
    std::cout << "\n";
  }

  // Move cursor to beginning of draw buffer
  const auto ui_height_lines = _ui_elements.size();
  std::cout << "\u001b[1000D"
            << "\u001b[" << ui_height_lines << "A";

  std::cout.flush();

  util::print_lock().unlock();
}

void TerminalUI::rebuild_progress_ui() {
  _ui_elements.clear();

  auto &progress_reporter = _state->get_progress_reporter();
  const auto &progress_counters = progress_reporter.get_progress_counters();

  if (progress_counters.empty())
    return;

  const auto longest_name = std::max_element(
      std::begin(progress_counters), std::end(progress_counters),
      [](const auto &l, const auto &r) {
        return l.first.size() < r.first.size();
      });
  const auto max_name_length = longest_name->first.size();

  for (auto &kv : progress_reporter.get_progress_counters()) {
    const auto &progress_name = kv.first;
    const auto &progress_counter = *kv.second;
    const auto progress = std::visit(
        [](const auto &counter) -> double {
          const auto cur_progress = counter.get_current_progress();
          const auto max_progress = counter.get_max_progress();
          return static_cast<double>(cur_progress) /
                 static_cast<double>(max_progress);
        },
        progress_counter);

    // Each progress counter has a name, a progress bar and a numerical
    // indicator of the progress

    auto progress_name_label = std::make_unique<TerminalLabel>();
    progress_name_label->set_content(
        format_progress_name(progress_name, max_name_length));
    progress_name_label->set_color(ForegroundColorWhite);

    auto progress_bar = std::make_unique<TerminalProgressBar>();
    progress_bar->set_progress(static_cast<float>(progress));
    progress_bar->set_allowed_width(40);

    auto progress_detail = std::make_unique<TerminalLabel>();
    progress_detail->set_content(format_progress_counter(progress_counter));
    progress_detail->set_color(ForegroundColorWhite);

    std::vector<std::unique_ptr<TerminalUIElement>> ui_for_progress_counter;
    ui_for_progress_counter.push_back(std::move(progress_name_label));
    ui_for_progress_counter.push_back(std::move(progress_bar));
    ui_for_progress_counter.push_back(std::move(progress_detail));

    _ui_elements.push_back(std::move(ui_for_progress_counter));
  }
}