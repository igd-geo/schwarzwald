#pragma once

#include <cstddef>
#include <chrono>
#include <vector>
#include <string>
#include <list>

#include <terminalpp/terminalpp.hpp>
#include <terminalpp/ansi_terminal.hpp>

namespace Potree {

    struct UIState {
        UIState();
        
        size_t get_processed_points() const;
        void set_processed_points(size_t processed_points);

        size_t get_total_points() const;
        void set_total_points(size_t total_points);

        float get_points_per_second() const;
        void set_points_per_second(float points_per_second);

        float get_progress() const;
        void set_progress(float progress);

        const std::string& get_current_mode() const;
        void set_current_mode(const std::string& mode);

        const std::list<std::string>& get_messages() const;
        void push_message(std::string message);

        constexpr static size_t MAX_MESSAGES = 6;

    private:
        //TODO Maybe make data members thread-safe
        size_t _processed_points;
        size_t _total_points;
        float _points_per_second;
        float _progress;
        std::string _current_mode;

        std::list<std::string> _messages;
    };

    struct TerminalRenderBuffer {

        const terminalpp::element& at(uint32_t x, uint32_t y) const;

        std::vector<terminalpp::element> elements;
        uint32_t width;
        uint32_t height;
    };

    enum class TerminalLayoutType {
        FixedWidth,
        Flexible
    };

    struct TerminalUIElement {
        virtual ~TerminalUIElement();

        virtual TerminalRenderBuffer render() const = 0;

        TerminalLayoutType get_layout_type() const;
    protected: 
        TerminalLayoutType _layout_type;
    };

    struct TerminalLabel : TerminalUIElement {
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

    struct TerminalMultilineLabel : TerminalLabel {
        TerminalMultilineLabel();
        virtual ~TerminalMultilineLabel();

        virtual TerminalRenderBuffer render() const override;

        void set_line_width(uint32_t line_width);

    private:
        uint32_t _line_width;
    };

    struct TerminalProgressBar : TerminalUIElement {
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

    struct TerminalUI {
        explicit TerminalUI(UIState const* state);

        void redraw();
    private:
        constexpr static std::chrono::milliseconds TERMINAL_REDRAW_INTERVAL = std::chrono::milliseconds{50};
        std::chrono::high_resolution_clock::time_point _last_redraw_time;

        UIState const* _state; 

        terminalpp::ansi_terminal _terminal;
        terminalpp::screen _screen;
        terminalpp::canvas _canvas;

        TerminalLabel _header_label;
        TerminalLabel _current_mode_label;

        TerminalProgressBar _main_progress_bar;
        TerminalLabel _main_progress_label;

        TerminalLabel _point_count_label;
        TerminalLabel _points_per_second_label;

        TerminalLabel _log_label;
    };

}