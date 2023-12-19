#ifndef CURAN_MUTATING_TEXTPANEL_HEADER_FILE_
#define CURAN_MUTATING_TEXTPANEL_HEADER_FILE_

#include "userinterface/widgets/ConfigDraw.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "include/core/SkFontMgr.h"
#include "skplaintexteditor/editor.h"

#include <cfloat>
#include <fstream>
#include <memory>

namespace curan
{
    namespace ui
    {

        class ImutableTextPanel : public curan::ui::Drawable, public curan::utilities::Lockable
        {
            const std::array<std::string, 3> kTypefaces = {"sans-serif", "serif", "monospace"};
            const size_t kTypefaceCount = kTypefaces.size();

            static constexpr SkFontStyle::Weight kFontWeight = SkFontStyle::kNormal_Weight;
            static constexpr SkFontStyle::Width kFontWidth = SkFontStyle::kNormal_Width;
            static constexpr SkFontStyle::Slant kFontSlant = SkFontStyle::kUpright_Slant;

            SkString fPath;
            std::vector<char> fClipboard;
            SkPlainTextEditor::Editor fEditor;
            SkPlainTextEditor::Editor::TextPosition fTextPos{0, 0};
            SkPlainTextEditor::Editor::TextPosition fMarkPos;
            int fPos = 0;
            int fWidth = 0;
            int fHeight = 0;
            int fMargin = 10;
            size_t fTypefaceIndex = 0;
            float fFontSize = 24;
            bool fShiftDown = false;

            bool fMouseDown = false;

            SkColor4f text_color;
            SkColor4f background_color;
            SkColor4f selection_color;

            ImutableTextPanel(const std::string& default_text);
        public:
            enum typeface
            {
                sans_serif = 0,
                serif = 1,
                monospace = 2
            };
            static std::unique_ptr<ImutableTextPanel> make(const std::string& default_text);

            void compile() override;

            ~ImutableTextPanel();

            curan::ui::drawablefunction draw() override;

            curan::ui::callablefunction call() override;

            void framebuffer_resize(const SkRect &new_page_size) override;

            void setFont(typeface font);

            bool moveCursor(SkPlainTextEditor::Editor::Movement m, bool shift = false);

            bool move(SkPlainTextEditor::Editor::TextPosition pos, bool shift);

            inline SkColor4f get_text_color()
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                return text_color;
            }

            inline ImutableTextPanel &set_text_color(SkColor4f color)
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

            inline ImutableTextPanel &set_background_color(SkColor4f color)
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                background_color = color;
                return *(this);
            }
        };

    }
}

#endif