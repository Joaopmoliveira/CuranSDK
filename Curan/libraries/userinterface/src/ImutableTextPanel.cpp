#include "userinterface/widgets/ImutableTextPanel.h"

namespace curan
{
    namespace ui
    {
        ImutableTextPanel::ImutableTextPanel(const std::string &in_default_text)
        {
            auto font_manager = SkFontMgr::RefDefault();
            fEditor.setFontMgr(font_manager);
            fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0,0},in_default_text.data(),in_default_text.size());
        }

        std::unique_ptr<ImutableTextPanel> ImutableTextPanel::make(const std::string &in_default_text)
        {
            std::unique_ptr<ImutableTextPanel> editor = std::unique_ptr<ImutableTextPanel>(new ImutableTextPanel(in_default_text));
            return editor;
        }

        void ImutableTextPanel::compile()
        {
        }

        ImutableTextPanel::~ImutableTextPanel()
        {
        }

        curan::ui::drawablefunction ImutableTextPanel::draw()
        {
            auto lamb = [this](SkCanvas *canvas)
            {
                SkAutoCanvasRestore acr(canvas, true);
                canvas->clipRect(get_position());
                canvas->translate(fMargin, (float)(fMargin - fPos));
                SkPlainTextEditor::Editor::PaintOpts options;
                options.fCursor = fTextPos;
                options.fCursorColor = SkColors::kTransparent;
                options.fBackgroundColor = get_background_color();
                options.fForegroundColor = get_text_color();
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

    curan::ui::callablefunction ImutableTextPanel::call()
    {
        auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
        {
            return false;
        };
        return lamb;
    }

    void ImutableTextPanel::framebuffer_resize(const SkRect &new_page_size)
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

    void ImutableTextPanel::setFont(typeface font)
    {
        fTypefaceIndex = font;
        fEditor.setFont(SkFont(fEditor.FontMgr()->matchFamilyStyle(kTypefaces[fTypefaceIndex].data(), SkFontStyle(kFontWeight, kFontWidth, kFontSlant)), fFontSize));
    }
}
}