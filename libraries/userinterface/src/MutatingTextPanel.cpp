#include "userinterface/widgets/MutatingTextPanel.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include <sstream>

namespace curan
{
    namespace ui
    {

        MutatingTextPanel::MutatingTextPanel(bool is_tight) : fTight{is_tight}
        {
            auto font_manager = fontMgr();
            fEditor.setFontMgr(font_manager);
        }

        MutatingTextPanel::MutatingTextPanel(bool is_tight,const std::string &in_default_text) : default_text{in_default_text}, fTight{is_tight}
        {
            auto font_manager = SkFontMgr::RefEmpty();
            fEditor.setFontMgr(font_manager);
            has_text_from_user = false;
            if (default_text)
            {
                fEditor.insert(fTextPos, (*default_text).data(), (*default_text).size());
            }
        }



        void MutatingTextPanel::compile()
        {
        }

        MutatingTextPanel::~MutatingTextPanel()
        {
        }

        curan::ui::drawablefunction MutatingTextPanel::draw()
        {
            if (fTight)
            {
                auto lamb = [this](SkCanvas *canvas)
                {
                    ++counter;
                    fBlink = (is_highlighted) ? counter % 30 < 15 : true;
                    SkAutoCanvasRestore acr(canvas, true);
                    SkPaint paint;
                    paint.setStyle(SkPaint::kFill_Style);
                    paint.setAntiAlias(true);
                    paint.setStrokeWidth(4);
                    paint.setColor((is_highlighted) ? get_highlighted_state() : get_background_color());
                    auto drawable = get_drawable_content();
                    drawable.offset(fMargin,fMargin);
                    canvas->drawRect(drawable, paint);
                    canvas->clipRect(get_position());
                    canvas->translate(drawable.x(), (float)(drawable.y() - fPos));
                    SkPlainTextEditor::Editor::PaintOpts options;
                    options.fCursor = fTextPos;
                    options.fCursorColor = get_cursor_color();
                    options.fCursorColor.fA = fBlink ? 0.0f : 1.0f;
                    options.fBackgroundColor = SkColors::kTransparent;
                    options.fForegroundColor = (has_text_from_user) ? get_text_color() : get_default_text_color();
                    if (fMarkPos != SkPlainTextEditor::Editor::TextPosition())
                    {
                        options.fSelectionBegin = fMarkPos;
                        options.fSelectionEnd = fTextPos;
                    }
                    {
                        std::lock_guard<std::mutex> g{get_mutex()};
                        fEditor.paint(canvas, options);
                    }
                };
                return lamb;
            }
            else
            {
                auto lamb = [this](SkCanvas *canvas)
                {
                    ++counter;
                    fBlink = (is_highlighted) ? counter % 30 < 15 : true;
                    SkAutoCanvasRestore acr(canvas, true);
                    SkPaint paint;
                    paint.setStyle(SkPaint::kFill_Style);
                    paint.setAntiAlias(true);
                    paint.setStrokeWidth(4);
                    paint.setColor((is_highlighted) ? get_highlighted_state() : get_background_color());
                    SkRect val = get_position();
                    val.offset(fMargin, fMargin);
                    canvas->drawRect(val, paint);
                    canvas->clipRect(get_position());
                    canvas->translate(val.x(), (float)(val.top() - fPos));
                    SkPlainTextEditor::Editor::PaintOpts options;
                    options.fCursor = fTextPos;
                    options.fCursorColor = get_cursor_color();
                    options.fCursorColor.fA = fBlink ? 0.0f : 1.0f;
                    options.fBackgroundColor = SkColors::kTransparent;
                    options.fForegroundColor = (has_text_from_user) ? get_text_color() : get_default_text_color();
                    if (fMarkPos != SkPlainTextEditor::Editor::TextPosition())
                    {
                        options.fSelectionBegin = fMarkPos;
                        options.fSelectionEnd = fTextPos;
                    }
                    {
                        std::lock_guard<std::mutex> g{get_mutex()};
                        fEditor.paint(canvas, options);
                    }
                };
                return lamb;
            }
        }

        curan::ui::callablefunction MutatingTextPanel::call()
        {
            if (fTight)
            {
                auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
                {
                    bool interacted = false;
                    std::visit(curan::utilities::overloaded{[this, &config](curan::ui::Empty arg) {

                                                            },
                                                            [this, &config, &interacted](curan::ui::Move arg)
                                                            {
                                                                if (!get_drawable_content().contains(arg.xpos, arg.ypos))
                                                                {
                                                                    fMouseDown = false;
                                                                    is_highlighted = false;
                                                                }
                                                                else
                                                                {
                                                                    is_highlighted = true;
                                                                }
                                                            },
                                                            [this, &config, &interacted](curan::ui::Press arg)
                                                            {
                                                                if (get_drawable_content().contains(arg.xpos, arg.ypos))
                                                                {
                                                                    fMouseDown = true;
                                                                    if (has_text_from_user)
                                                                    {
                                                                        fTextPos = fEditor.getPosition(SkIPoint::Make(arg.xpos - get_drawable_content().x(), arg.ypos - get_drawable_content().y()));
                                                                    }

                                                                    is_highlighted = true;
                                                                    interacted = true;
                                                                }
                                                            },
                                                            [this, &config, &interacted](curan::ui::Scroll arg)
                                                            {
                                                                if (has_text_from_user)
                                                                {
                                                                    int delta = arg.yoffset * fEditor.font().getSpacing();
                                                                    int maxPos = std::max(0, fEditor.getHeight() + 2 * fMargin - fHeight / 2);
                                                                    int newpos = std::max(0, std::min(fPos + delta, maxPos));
                                                                    if (newpos != fPos)
                                                                        fPos = newpos;
                                                                    interacted = true;
                                                                }
                                                            },
                                                            [this, &config](curan::ui::Unpress arg)
                                                            {
                                                                fMouseDown = false;
                                                            },
                                                            [this, &config, &interacted](curan::ui::Key arg)
                                                            {
                                                                if (!is_highlighted)
                                                                    return;
                                                                switch (arg.key_type)
                                                                {
                                                                case curan::ui::ControlKeys::_NOT:
                                                                    repeated_deleting = 0;
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.insert(fTextPos, &arg.ascii_version, sizeof(arg.ascii_version));
                                                                        else
                                                                        {
                                                                            SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
                                                                            fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
                                                                            fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, &arg.ascii_version, sizeof(arg.ascii_version));
                                                                            has_text_from_user = true;
                                                                        }
                                                                        interacted = true;
                                                                    }

                                                                    break;
                                                                case curan::ui::ControlKeys::_DELETE:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_BACKSPACE:
                                                                {
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (!has_text_from_user)
                                                                        {
                                                                            ++repeated_deleting;
                                                                            return;
                                                                        }
                                                                        repeated_deleting = 0;
                                                                        auto pos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        fTextPos = fEditor.remove(fTextPos, pos);
                                                                        if (fTextPos == SkPlainTextEditor::Editor::TextPosition{0, 0})
                                                                        {
                                                                            has_text_from_user = false;
                                                                            if (default_text)
                                                                            {
                                                                                fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, (*default_text).data(), (*default_text).size());
                                                                            }
                                                                        }
                                                                        interacted = true;
                                                                        ++repeated_deleting;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (!has_text_from_user)
                                                                        {
                                                                            ++repeated_deleting;
                                                                            return;
                                                                        }
                                                                        auto pos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        move(fEditor.remove(fTextPos, pos), false);
                                                                        if (fTextPos == SkPlainTextEditor::Editor::TextPosition{0, 0})
                                                                        {
                                                                            has_text_from_user = false;
                                                                            if (default_text)
                                                                            {
                                                                                fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, (*default_text).data(), (*default_text).size());
                                                                            }
                                                                        }
                                                                        interacted = true;
                                                                        ++repeated_deleting;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_DOWN:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kDown, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kDown, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_ENTER:
                                                                {
                                                                    repeated_deleting = 0;
                                                                    if (arg.action == GLFW_PRESS && has_text_from_user)
                                                                    {   
                                                                        interacted = true;
                                                                        std::stringstream ss;
                                                                        for(const auto s : fEditor.text())
                                                                            ss << std::string{s.data,s.size};
                                                                        for(const auto& call : panel_callback){
                                                                            call(this,ss.str(),config);
                                                                        }
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_ESCAPE:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_INSERT:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_LEFT:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_PAGE_DOWN:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_PAGE_UP:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_RIGHT:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kRight, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kRight, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_TAB:
                                                                {
                                                                    repeated_deleting = 0;
                                                                    const char enter = '\t';
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.insert(fTextPos, &enter, sizeof(enter));
                                                                        else
                                                                        {
                                                                            SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
                                                                            fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
                                                                            fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, &enter, sizeof(enter));
                                                                            has_text_from_user = true;
                                                                        }
                                                                        interacted = true;
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_UP:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kUp, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kUp, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                default:
                                                                    repeated_deleting = 0;
                                                                    return;
                                                                }
                                                            },
                                                            [this, &config](curan::ui::ItemDropped arg) {

                                                            }},
                               sig);
                    return interacted;
                };
                return lamb;
            }
            else
            {
                auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
                {
                    bool interacted = false;
                    std::visit(curan::utilities::overloaded{[this, &config](curan::ui::Empty arg) {

                                                            },
                                                            [this, &config, &interacted](curan::ui::Move arg)
                                                            {
                                                                if (!get_position().contains(arg.xpos, arg.ypos))
                                                                {
                                                                    fMouseDown = false;
                                                                    is_highlighted = false;
                                                                }
                                                                else
                                                                {
                                                                    is_highlighted = true;
                                                                }
                                                            },
                                                            [this, &config, &interacted](curan::ui::Press arg)
                                                            {
                                                                if (get_position().contains(arg.xpos, arg.ypos))
                                                                {
                                                                    fMouseDown = true;
                                                                    if (has_text_from_user)
                                                                    {
                                                                        fTextPos = fEditor.getPosition(SkIPoint::Make(arg.xpos - get_position().x(), arg.ypos - get_position().y()));
                                                                    }

                                                                    is_highlighted = true;
                                                                    interacted = true;
                                                                }
                                                            },
                                                            [this, &config, &interacted](curan::ui::Scroll arg)
                                                            {
                                                                if (has_text_from_user)
                                                                {
                                                                    int delta = arg.yoffset * fEditor.font().getSpacing();
                                                                    int maxPos = std::max(0, fEditor.getHeight() + 2 * fMargin - fHeight / 2);
                                                                    int newpos = std::max(0, std::min(fPos + delta, maxPos));
                                                                    if (newpos != fPos)
                                                                        fPos = newpos;
                                                                    interacted = true;
                                                                }
                                                            },
                                                            [this, &config](curan::ui::Unpress arg)
                                                            {
                                                                fMouseDown = false;
                                                            },
                                                            [this, &config, &interacted](curan::ui::Key arg)
                                                            {
                                                                if (!is_highlighted)
                                                                    return;
                                                                switch (arg.key_type)
                                                                {
                                                                case curan::ui::ControlKeys::_NOT:
                                                                    repeated_deleting = 0;
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.insert(fTextPos, &arg.ascii_version, sizeof(arg.ascii_version));
                                                                        else
                                                                        {
                                                                            SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
                                                                            fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
                                                                            fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, &arg.ascii_version, sizeof(arg.ascii_version));
                                                                            has_text_from_user = true;
                                                                        }
                                                                        interacted = true;
                                                                    }

                                                                    break;
                                                                case curan::ui::ControlKeys::_DELETE:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_BACKSPACE:
                                                                {
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (!has_text_from_user)
                                                                        {
                                                                            ++repeated_deleting;
                                                                            return;
                                                                        }
                                                                        repeated_deleting = 0;
                                                                        auto pos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        fTextPos = fEditor.remove(fTextPos, pos);
                                                                        if (fTextPos == SkPlainTextEditor::Editor::TextPosition{0, 0})
                                                                        {
                                                                            has_text_from_user = false;
                                                                            if (default_text)
                                                                            {
                                                                                fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, (*default_text).data(), (*default_text).size());
                                                                            }
                                                                        }
                                                                        interacted = true;
                                                                        ++repeated_deleting;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (!has_text_from_user)
                                                                        {
                                                                            ++repeated_deleting;
                                                                            return;
                                                                        }
                                                                        auto pos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        move(fEditor.remove(fTextPos, pos), false);
                                                                        if (fTextPos == SkPlainTextEditor::Editor::TextPosition{0, 0})
                                                                        {
                                                                            has_text_from_user = false;
                                                                            if (default_text)
                                                                            {
                                                                                fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, (*default_text).data(), (*default_text).size());
                                                                            }
                                                                        }
                                                                        interacted = true;
                                                                        ++repeated_deleting;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_DOWN:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kDown, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kDown, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_ENTER:
                                                                {
                                                                    repeated_deleting = 0;
                                                                    const char enter = '\n';
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.insert(fTextPos, &enter, sizeof(enter));
                                                                        else
                                                                        {
                                                                            SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
                                                                            fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
                                                                            fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, &enter, sizeof(enter));
                                                                            has_text_from_user = true;
                                                                        }
                                                                        interacted = true;
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_ESCAPE:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_INSERT:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_LEFT:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kLeft, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_PAGE_DOWN:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_PAGE_UP:
                                                                    repeated_deleting = 0;
                                                                    break;
                                                                case curan::ui::ControlKeys::_RIGHT:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kRight, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kRight, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                case curan::ui::ControlKeys::_TAB:
                                                                {
                                                                    repeated_deleting = 0;
                                                                    const char enter = '\t';
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.insert(fTextPos, &enter, sizeof(enter));
                                                                        else
                                                                        {
                                                                            SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
                                                                            fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
                                                                            fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, &enter, sizeof(enter));
                                                                            has_text_from_user = true;
                                                                        }
                                                                        interacted = true;
                                                                    }
                                                                }
                                                                break;
                                                                case curan::ui::ControlKeys::_UP:
                                                                    if (arg.action == GLFW_PRESS)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kUp, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (repeated_deleting > 5)
                                                                    {
                                                                        if (has_text_from_user)
                                                                            fTextPos = fEditor.move(SkPlainTextEditor::Editor::Movement::kUp, fTextPos);
                                                                        ++repeated_deleting;
                                                                        interacted = true;
                                                                    }
                                                                    else if (GLFW_RELEASE)
                                                                    {
                                                                        repeated_deleting = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        ++repeated_deleting;
                                                                    }
                                                                    break;
                                                                default:
                                                                    repeated_deleting = 0;
                                                                    return;
                                                                }
                                                            },
                                                            [this, &config](curan::ui::ItemDropped arg) {

                                                            }},
                               sig);
                    return interacted;
                };
                return lamb;
            }
        }

        void MutatingTextPanel::framebuffer_resize(const SkRect &new_page_size)
        {
            if (fTight)
            {
                auto pos = get_drawable_content();
                if (SkISize{fWidth, fHeight} != SkISize{(int)pos.width(), (int)pos.height()})
                {
                    fHeight = (int)pos.height();
                    if (pos.width() != fWidth)
                    {
                        fWidth = (int)pos.width();
                        fEditor.setWidth(fWidth - 2 * fMargin);
                    }
                }
            }
            else
            {
                auto pos = get_position();
                if (SkISize{fWidth, fHeight} != SkISize{(int)pos.width(), (int)pos.height()})
                {
                    fHeight = (int)pos.height();
                    if (pos.width() != fWidth)
                    {
                        fWidth = (int)pos.width();
                        fEditor.setWidth(fWidth - 2 * fMargin);
                    }
                }
            }
        }

        void MutatingTextPanel::setFont(typeface font)
        {
            fTypefaceIndex = font;
            fEditor.setFont(SkFont(fEditor.FontMgr()->matchFamilyStyle(kTypefaces[fTypefaceIndex].data(), SkFontStyle(kFontWeight, kFontWidth, kFontSlant)), fFontSize));
        }

        void MutatingTextPanel::appendtext(const std::string &in_text)
        {
            if (has_text_from_user)
                fTextPos = fEditor.insert(fTextPos, in_text.data(), in_text.size());
            else
            {
                fTextPos = fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, in_text.data(), in_text.size());
                has_text_from_user = true;
            }
        }

        void MutatingTextPanel::replacetext(const std::string &in_text)
        {
            SkPlainTextEditor::Editor::TextPosition replacement_at_start{0, 0};
            fTextPos = fEditor.insert(replacement_at_start, in_text.data(), in_text.size());
            has_text_from_user = true;
        }

        bool MutatingTextPanel::moveCursor(SkPlainTextEditor::Editor::Movement m, bool shift)
        {
            return this->move(fEditor.move(m, fTextPos), shift);
        }

        bool MutatingTextPanel::move(SkPlainTextEditor::Editor::TextPosition pos, bool shift)
        {
            if (pos == fTextPos || pos == SkPlainTextEditor::Editor::TextPosition())
            {
                if (!shift)
                {
                    fMarkPos = SkPlainTextEditor::Editor::TextPosition();
                }
                return false;
            }
            if (shift != fShiftDown)
            {
                fMarkPos = shift ? fTextPos : SkPlainTextEditor::Editor::TextPosition();
                fShiftDown = shift;
            }
            fTextPos = pos;

            // scroll if needed.
            SkIRect cursor = fEditor.getLocation(fTextPos).roundOut();
            if (fPos < cursor.bottom() - fHeight + 2 * fMargin)
            {
                fPos = cursor.bottom() - fHeight + 2 * fMargin;
            }
            else if (cursor.top() < fPos)
            {
                fPos = cursor.top();
            }
            return true;
        }
    }
}