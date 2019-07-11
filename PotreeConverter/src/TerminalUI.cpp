#include "TerminalUI.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <array>

using namespace std::chrono;


static void terminal_blit(const Potree::TerminalRenderBuffer& source, terminalpp::canvas& target, uint32_t target_x, uint32_t target_y) {
    if(target_x >= static_cast<uint32_t>(target.size().width) || target_y >= static_cast<uint32_t>(target.size().height)) return;

    const auto end_x = std::min(target_x + source.width, static_cast<uint32_t>(target.size().width));
    const auto end_y = std::min(target_y + source.height, static_cast<uint32_t>(target.size().height));

    for(auto y = target_y; y < end_y; ++y) {
        for(auto x = target_x; x < end_x; ++x) {
            target[x][y] = source.at(x - target_x, y - target_y);
        }
    }
}

static void terminal_canvas_clear(terminalpp::canvas& canvas) {
    for(auto y = 0; y < canvas.size().height; ++y) {
        for(auto x = 0; x < canvas.size().width; ++x) {
            canvas[x][y] = {};
        }
    }
}

static std::string pretty_print_large_number(double large_number) {
    constexpr std::array<const char*, 5> suffixes = {"", "K", "M", "B", "T"};

    const auto order_of_magnitude = [=](double val) {
        double cur_val = val;
        for(size_t order = 0; order < 4; ++order) {
            if(cur_val < 1000) return order;
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

static std::string format_point_counts(size_t processed_count, size_t total_count) {
    std::stringstream ss;

    ss << std::fixed << std::setprecision(1) << pretty_print_large_number(processed_count) << "/" << pretty_print_large_number(total_count) << " points";
    return ss.str();
}

static std::string format_points_per_second(float points_per_second) {
    std::stringstream ss;
    ss << " | " << pretty_print_large_number(points_per_second) << " points/s";
    return ss.str();
}


Potree::UIState::UIState() : 
    _processed_points(0),
    _total_points(0),
    _points_per_second(0),
    _progress(0) {}

size_t Potree::UIState::get_processed_points() const {
    return _processed_points;
}

void Potree::UIState::set_processed_points(size_t processed_points) {
    _processed_points = processed_points;
}

size_t Potree::UIState::get_total_points() const {
    return _total_points;
}

void Potree::UIState::set_total_points(size_t total_points) {
    _total_points = total_points;
}

float Potree::UIState::get_points_per_second() const {
    return _points_per_second;
}

void Potree::UIState::set_points_per_second(float points_per_second) {
    _points_per_second = points_per_second;
}

float Potree::UIState::get_progress() const { return _progress; }

void Potree::UIState::set_progress(float progress) {
    _progress = progress;
}

const std::string& Potree::UIState::get_current_mode() const {
    return _current_mode;
}

void Potree::UIState::set_current_mode(const std::string& mode) {
    _current_mode = mode;
}

const std::list<std::string>& Potree::UIState::get_messages() const {
    return _messages;
}

void Potree::UIState::push_message(std::string message) {
    _messages.push_back(std::move(message));
    if(_messages.size() > MAX_MESSAGES) {
        _messages.pop_front();
    }
}


const terminalpp::element& Potree::TerminalRenderBuffer::at(uint32_t x, uint32_t y) const {
    return elements[y * width + x];
}


Potree::TerminalUIElement::~TerminalUIElement() {}

Potree::TerminalLayoutType Potree::TerminalUIElement::get_layout_type() const {
    return _layout_type;
}


Potree::TerminalLabel::TerminalLabel() {
    _layout_type = TerminalLayoutType::FixedWidth;
}

Potree::TerminalLabel::~TerminalLabel() {}

Potree::TerminalRenderBuffer Potree::TerminalLabel::render() const {
    Potree::TerminalRenderBuffer render_buffer;
    render_buffer.width = _content.size();
    render_buffer.height = 1;
    render_buffer.elements.reserve(_content.size());

    for(auto character : _content) {
        terminalpp::element elem{character};
        elem.attribute_.foreground_colour_ = _color;
        render_buffer.elements.push_back(std::move(elem));
    }

    return render_buffer;
}

const std::string& Potree::TerminalLabel::get_content() const {
    return _content;
}

const terminalpp::colour& Potree::TerminalLabel::get_color() const {
    return _color;
}

void Potree::TerminalLabel::set_content(const std::string& content) {
    _content = content;
}

void Potree::TerminalLabel::set_color(const terminalpp::colour& color) {
    _color = color;
}


Potree::TerminalMultilineLabel::TerminalMultilineLabel() :
    _line_width(80) {
}

Potree::TerminalMultilineLabel::~TerminalMultilineLabel() {}

Potree::TerminalRenderBuffer Potree::TerminalMultilineLabel::render() const {

    const auto chunk_string = [](const std::string& str, uint32_t chunk_length) -> std::vector<std::string> {
        const auto num_chunks = static_cast<size_t>( std::ceil(str.size() / static_cast<double>(chunk_length)) );
        std::vector<std::string> chunks;
        chunks.reserve(num_chunks);
        for(size_t chunk = 0; chunk < num_chunks; ++chunk) {
            const auto start_iter = str.begin() + (chunk * chunk_length);
            const auto end_iter = ((chunk + 1) * chunk_length >= str.size()) ? str.end() : str.begin() + ((chunk + 1) * chunk_length);
            chunks.emplace_back(start_iter, end_iter);
        }

        return chunks;
    };

    const auto lines = chunk_string(get_content(), _line_width);
    const auto max_line_length = std::max_element(lines.begin(), lines.end(), [](const auto& l, const auto& r) {
        return l.size() < r.size();
    })->size();

    TerminalRenderBuffer render_buffer;
    render_buffer.width = static_cast<uint32_t>(max_line_length);
    render_buffer.height = static_cast<uint32_t>(lines.size());
    render_buffer.elements.resize(render_buffer.width * render_buffer.height);

    for(size_t line_idx = 0; line_idx < lines.size(); ++line_idx) {
        const auto& cur_line = lines[line_idx];
        for(size_t char_idx = 0; char_idx < cur_line.size(); ++char_idx) {
            terminalpp::element elem{cur_line[char_idx]};
            elem.attribute_.foreground_colour_ = get_color();
            render_buffer.elements[line_idx * max_line_length + char_idx] = std::move(elem);
        }
    }

    return render_buffer;
}

void Potree::TerminalMultilineLabel::set_line_width(uint32_t line_width) {
    _line_width = line_width;
}


Potree::TerminalProgressBar::TerminalProgressBar() :
    _progress(0),
    _allowed_width(0) {
        _layout_type = TerminalLayoutType::Flexible;
    }

Potree::TerminalProgressBar::~TerminalProgressBar() {}

Potree::TerminalRenderBuffer Potree::TerminalProgressBar::render() const {
    if(_allowed_width < 3) return {};

    const auto steps = _allowed_width - 2;
    const auto enabled_steps = std::floor(steps * _progress);

    Potree::TerminalRenderBuffer render_buffer;
    render_buffer.width = _allowed_width;
    render_buffer.height = 1;
    render_buffer.elements.reserve(_allowed_width);

    //TODO Colors

    const auto filled_glyph = terminalpp::glyph{"*"};
    const auto empty_glyph = terminalpp::glyph{" "};

    render_buffer.elements.push_back(terminalpp::element{'['});
    for(uint32_t idx = 0; idx < steps; ++idx) {
        if(idx < enabled_steps) { 
            render_buffer.elements.push_back(terminalpp::element{filled_glyph});
        } else {
            render_buffer.elements.push_back(terminalpp::element{empty_glyph});
        }
    }
    render_buffer.elements.push_back(terminalpp::element{']'});

    return render_buffer;
}

float Potree::TerminalProgressBar::get_progress() const {
    return _progress;
}

void Potree::TerminalProgressBar::set_progress(float progress) {
    _progress = progress;
}

void Potree::TerminalProgressBar::set_allowed_width(uint32_t allowed_width) {
    _allowed_width = allowed_width;
}


Potree::TerminalUI::TerminalUI(Potree::UIState const* state) : 
    _state(state),
    _canvas({100, 24}) {
        
        _header_label.set_content("3D tiles pointcloud converter -- v0.1");
        _header_label.set_color(terminalpp::high_colour{5,5,5});

        _current_mode_label.set_content("Currently: INDEXING");
        _current_mode_label.set_color(terminalpp::high_colour{5,5,5});

        _main_progress_bar.set_progress(0.f);
        _main_progress_bar.set_allowed_width(80);
        _main_progress_label.set_content("0%");
        _main_progress_label.set_color(terminalpp::high_colour{5,5,5});

        _log_label.set_content("Log:");
        _log_label.set_color(terminalpp::high_colour{5,5,5});
    }

void Potree::TerminalUI::redraw() {
    const auto time_since_last_redraw = duration_cast<milliseconds>(high_resolution_clock::now() - _last_redraw_time);
    if(time_since_last_redraw < TERMINAL_REDRAW_INTERVAL) return;
    _last_redraw_time = high_resolution_clock::now();

    //Update elements from UI_state
    std::stringstream ss;
    ss << " " << std::fixed << std::setprecision(2) << (100 * _state->get_progress()) << "%";
    _main_progress_label.set_content(ss.str());

    _main_progress_bar.set_progress(_state->get_progress());

    ss = {};
    ss << "Currently: " << _state->get_current_mode();
    _current_mode_label.set_content(ss.str());

    _point_count_label.set_content(format_point_counts(_state->get_processed_points(), _state->get_total_points()));
    _points_per_second_label.set_content(format_points_per_second(_state->get_points_per_second()));

    //Redraw
    terminal_canvas_clear(_canvas);

    const auto header_rendered = _header_label.render();
    terminal_blit(header_rendered, _canvas, 0, 0);

    const auto current_mode_rendered = _current_mode_label.render();
    terminal_blit(current_mode_rendered, _canvas, 0, 1);


    //Render progress bar and its label, the label is fixed-width, the progress bar is flexible, so the label
    //will trim the progress bar
    //TODO Write some layouting classes that handle this
    const auto progress_label_rendered = _main_progress_label.render();
    _main_progress_bar.set_allowed_width(_canvas.size().width - progress_label_rendered.width);
    const auto progress_bar_rendered = _main_progress_bar.render();
    terminal_blit(progress_bar_rendered, _canvas, 0, 2);
    terminal_blit(progress_label_rendered, _canvas, progress_bar_rendered.width, 2);

    const auto point_count_label_rendered = _point_count_label.render();
    terminal_blit(point_count_label_rendered, _canvas, 0, 3);

    const auto points_per_second_rendered = _points_per_second_label.render();
    terminal_blit(points_per_second_rendered, _canvas, point_count_label_rendered.width, 3);

    //Render log messages
    const auto log_label_rendered = _log_label.render();
    terminal_blit(log_label_rendered, _canvas, 0, 4);

    const auto& log_messages = _state->get_messages();
    uint32_t log_msg_idx = 5;
    for(auto& log_msg : log_messages) {
        TerminalMultilineLabel tmp_label;
        tmp_label.set_line_width(_canvas.size().width - 3);
        tmp_label.set_content(log_msg);
        tmp_label.set_color(terminalpp::high_colour{5,5,5});

        const auto msg_rendered = tmp_label.render();
        terminal_blit(msg_rendered, _canvas, 3, log_msg_idx++);
    }

    std::cout << _screen.draw(_terminal, _canvas);
    std::cout.flush();
}