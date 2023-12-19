#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include "utils/Overloading.h"
#include <iostream>
#include <thread>

// Copyright 2019 Google LLC.
// Use of this source code is governed by a BSD-style license that can be found in the LICENSE file.

// Proof of principle of a text editor written with Skia & SkShaper.
// https://bugs.skia.org/9020

#include "include/core/SkFontMgr.h"

#include "skplaintexteditor/editor.h"

#include <cfloat>
#include <fstream>
#include <memory>

using SkPlainTextEditor::Editor;
using SkPlainTextEditor::StringView;

static constexpr float kFontSize = 30;
static const char *kTypefaces[3] = {"sans-serif", "serif", "monospace"};
static constexpr size_t kTypefaceCount = std::size(kTypefaces);

static constexpr SkFontStyle::Weight kFontWeight = SkFontStyle::kNormal_Weight;
static constexpr SkFontStyle::Width kFontWidth = SkFontStyle::kNormal_Width;
static constexpr SkFontStyle::Slant kFontSlant = SkFontStyle::kUpright_Slant;

// Note: initialization is not thread safe
sk_sp<SkFontMgr> fontMgr()
{
    static bool init = false;
    static sk_sp<SkFontMgr> fontMgr = nullptr;
    if (!init)
    {
#if defined(SK_FONTMGR_FONTCONFIG_AVAILABLE)
        fontMgr = SkFontMgr_New_FontConfig(nullptr);
#elif defined(SK_FONTMGR_CORETEXT_AVAILABLE)
        fontMgr = SkFontMgr_New_CoreText(nullptr);
#elif defined(SK_FONTMGR_DIRECTWRITE_AVAILABLE)
        fontMgr = SkFontMgr_New_DirectWrite();
#endif
        init = true;
    }
    return fontMgr;
}

#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"

static std::string gSpeach = "On the other hand, we denounce with righteous indignation and dislike men who are so beguiled and demoralized by the charms of pleasure of the moment, so blinded by desire, that they cannot foresee the pain and trouble that are bound to ensue; and equal blame belongs to those who fail in their duty through weakness of will, which is the same as saying through shrinking from toil and pain. These cases are perfectly simple and easy to distinguish. In a free hour, when our power of choice is untrammelled and when nothing prevents our being able to do what we like best, every pleasure is to be welcomed and every pain avoided. But in certain circumstances and owing to the claims of duty or the obligations of business it will frequently occur that pleasures have to be repudiated and annoyances accepted. The wise man therefore always holds in these matters to this principle of selection: he rejects pleasures to secure other greater pleasures, or else he endures pains to avoid worse pains.";

class MutatingTextPanel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<MutatingTextPanel>
{
    SkString fPath;
    std::vector<char> fClipboard;
    Editor fEditor;
    Editor::TextPosition fTextPos{0, 0};
    Editor::TextPosition fMarkPos;
    int fPos = 0;    // window pixel position in file
    int fWidth = 0;  // window width
    int fHeight = 0; // window height
    int fMargin = 10;
    size_t fTypefaceIndex = 0;
    float fFontSize = kFontSize;
    bool fShiftDown = false;
    bool fBlink = false;
    size_t repeated_deleting = 0;
    bool fMouseDown = false;
    size_t counter = 0;

    SkColor4f text_color;
    SkColor4f background_color;
    SkColor4f cursor_color;
    SkColor4f selection_color;

    MutatingTextPanel()
    {
        fEditor.setFontMgr(SkFontMgr::RefDefault());
    }

public:
    inline SkColor4f get_text_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return text_color;
    }

    inline MutatingTextPanel &set_text_color(SkColor4f color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        text_color = color;
        return *(this);
    }

    inline SkColor4f get_background_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return background_color;
    }

    inline MutatingTextPanel &set_background_color(SkColor4f color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        background_color = color;
        return *(this);
    }

    inline MutatingTextPanel &set_cursor_color(SkColor4f new_waiting_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        cursor_color = new_waiting_color;
        return *(this);
    }

    inline SkColor4f get_cursor_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return cursor_color;
    }

    inline MutatingTextPanel &set_selection_color(SkColor4f new_waiting_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        selection_color = new_waiting_color;
        return *(this);
    }

    inline SkColor4f get_selection_state()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return selection_color;
    }

    static std::unique_ptr<MutatingTextPanel> make()
    {
        std::unique_ptr<MutatingTextPanel> editor = std::unique_ptr<MutatingTextPanel>(new MutatingTextPanel());
        return editor;
    }

    void compile() override
    {
    }

    ~MutatingTextPanel()
    {
    }

    curan::ui::drawablefunction draw() override
    {
        auto lamb = [this](SkCanvas *canvas)
        {
            ++counter;
            fBlink = counter % 30 < 15;
            SkAutoCanvasRestore acr(canvas, true);
            canvas->clipRect(get_position());
            canvas->translate(fMargin, (float)(fMargin - fPos));
            Editor::PaintOpts options;
            options.fCursor = fTextPos;
            options.fCursorColor = get_cursor_color();
            options.fCursorColor.fA = fBlink ? 0.0f : 1.0f;
            options.fBackgroundColor = get_background_color();
            options.fForegroundColor = get_text_color();
            if (fMarkPos != Editor::TextPosition())
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

    curan::ui::callablefunction call() override
    {
        auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
        {
            bool interacted = false;
            std::visit(curan::utilities::overloaded{[this, &config](curan::ui::Empty arg) {

                                                    },
                                                    [this, &config](curan::ui::Move arg)
                                                    {
                                                        if (!get_position().contains(arg.xpos, arg.ypos))
                                                        {
                                                            fMouseDown = false;
                                                        }
                                                        // move(fEditor.move(SkPlainTextEditor::Editor::Movement::kWordLeft, fEditor.getPosition(SkIPoint::Make(arg.xpos, arg.ypos))), true);
                                                    },
                                                    [this, &config](curan::ui::Press arg)
                                                    {
                                                        if (get_position().contains(arg.xpos, arg.ypos))
                                                        {
                                                            fMouseDown = true;
                                                            fTextPos = fEditor.getPosition(SkIPoint::Make(arg.xpos, arg.ypos));
                                                        }
                                                    },
                                                    [this, &config](curan::ui::Scroll arg)
                                                    {
                                                        int delta = arg.yoffset * fEditor.font().getSpacing();
                                                        int maxPos = std::max(0, fEditor.getHeight() + 2 * fMargin - fHeight / 2);
                                                        int newpos = std::max(0, std::min(fPos + delta, maxPos));
                                                        if (newpos != fPos)
                                                            fPos = newpos;
                                                    },
                                                    [this, &config](curan::ui::Unpress arg)
                                                    {
                                                        fMouseDown = false;
                                                    },
                                                    [this, &config](curan::ui::Key arg)
                                                    {
                                                        switch (arg.key_type)
                                                        {
                                                        case curan::ui::ControlKeys::_NOT:
                                                            repeated_deleting = 0;
                                                            if (arg.action == GLFW_PRESS)
                                                                fTextPos = fEditor.insert(fTextPos, &arg.ascii_version, sizeof(arg.ascii_version));
                                                            break;
                                                        case curan::ui::ControlKeys::_DELETE:
                                                            repeated_deleting = 0;
                                                            break;
                                                        case curan::ui::ControlKeys::_BACKSPACE:
                                                        {
                                                            if (arg.action == GLFW_PRESS)
                                                            {
                                                                repeated_deleting = 0;
                                                                auto pos = fEditor.move(Editor::Movement::kLeft, fTextPos);
                                                                move(fEditor.remove(fTextPos, pos), false);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (repeated_deleting > 5)
                                                            {
                                                                auto pos = fEditor.move(Editor::Movement::kLeft, fTextPos);
                                                                move(fEditor.remove(fTextPos, pos), false);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (GLFW_RELEASE)
                                                            {
                                                                repeated_deleting = 0;
                                                            } else {
                                                                ++repeated_deleting;
                                                            }
                                                        }
                                                        break;
                                                        case curan::ui::ControlKeys::_DOWN:
                                                            if (arg.action == GLFW_PRESS)
                                                            {
                                                                repeated_deleting = 0;
                                                                fTextPos = fEditor.move(Editor::Movement::kDown, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (repeated_deleting > 5)
                                                            {
                                                                fTextPos = fEditor.move(Editor::Movement::kDown, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (GLFW_RELEASE)
                                                            {
                                                                repeated_deleting = 0;
                                                            } else {
                                                                ++repeated_deleting;
                                                            }
                                                            break;
                                                        case curan::ui::ControlKeys::_ENTER:
                                                        {
                                                            repeated_deleting = 0;
                                                            const char enter = '\n';
                                                            if (arg.action == GLFW_PRESS)
                                                                fTextPos = fEditor.insert(fTextPos, &enter, sizeof(enter));
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
                                                                fTextPos = fEditor.move(Editor::Movement::kLeft, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (repeated_deleting > 5)
                                                            {
                                                                fTextPos = fEditor.move(Editor::Movement::kLeft, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (GLFW_RELEASE)
                                                            {
                                                                repeated_deleting = 0;
                                                            } else {
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
                                                                fTextPos = fEditor.move(Editor::Movement::kRight, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (repeated_deleting > 5)
                                                            {
                                                                fTextPos = fEditor.move(Editor::Movement::kRight, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (GLFW_RELEASE)
                                                            {
                                                                repeated_deleting = 0;
                                                            } else {
                                                                ++repeated_deleting;
                                                            }
                                                            break;
                                                        case curan::ui::ControlKeys::_TAB:
                                                        {
                                                            repeated_deleting = 0;
                                                            const char enter = '\t';
                                                            if (arg.action == GLFW_PRESS)
                                                                fTextPos = fEditor.insert(fTextPos, &enter, sizeof(enter));
                                                        }
                                                        break;
                                                        case curan::ui::ControlKeys::_UP:
                                                            if (arg.action == GLFW_PRESS)
                                                            {
                                                                repeated_deleting = 0;
                                                                fTextPos = fEditor.move(Editor::Movement::kUp, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (repeated_deleting > 5)
                                                            {
                                                                fTextPos = fEditor.move(Editor::Movement::kUp, fTextPos);
                                                                ++repeated_deleting;
                                                            }
                                                            else if (GLFW_RELEASE)
                                                            {
                                                                repeated_deleting = 0;
                                                            } else {
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

    void framebuffer_resize(const SkRect &new_page_size) override
    {
        std::lock_guard<std::mutex> g{get_mutex()};
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

    void setFont()
    {
        fEditor.setFont(SkFont(fontMgr()->matchFamilyStyle(kTypefaces[fTypefaceIndex],
                                                           SkFontStyle(kFontWeight, kFontWidth, kFontSlant)),
                               fFontSize));
    }

    void loadtext()
    {
        fEditor.insert(Editor::TextPosition{0, 0}, gSpeach.data(), gSpeach.size());
    }

    bool moveCursor(Editor::Movement m, bool shift = false)
    {
        return this->move(fEditor.move(m, fTextPos), shift);
    }

    bool move(Editor::TextPosition pos, bool shift)
    {
        if (pos == fTextPos || pos == Editor::TextPosition())
        {
            if (!shift)
            {
                fMarkPos = Editor::TextPosition();
            }
            return false;
        }
        if (shift != fShiftDown)
        {
            fMarkPos = shift ? fTextPos : Editor::TextPosition();
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
};

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::unique_ptr<MutatingTextPanel> layer = MutatingTextPanel::make();
        layer->set_background_color({1.f,1.0f,1.0f,1.0f}).set_cursor_color({1.0f,.0f,.0f,1.0f}).set_text_color({.0f,.0f,.0f,1.0f});
        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        layer->loadtext();
        *container << std::move(layer);

        curan::ui::Page page{std::move(container), SK_ColorBLACK};

        ConfigDraw config{&page};

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();

            if (viewer->was_updated())
            {
                page.update_page(viewer.get());
                viewer->update_processed();
            }
            page.draw(canvas);
            auto signals = viewer->process_pending_signals();
            if (!signals.empty())
                page.propagate_signal(signals.back(), &config);
            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (...)
    {
        std::cout << "Failed";
        return 1;
    }
}