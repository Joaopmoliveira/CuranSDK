#ifndef CURAN_MUTATING_TEXTPANEL_HEADER_FILE_
#define CURAN_MUTATING_TEXTPANEL_HEADER_FILE_

#include "userinterface/widgets/ConfigDraw.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "include/core/SkFontMgr.h"
#include "skplaintexteditor/editor.h"

#include <cfloat>
#include <fstream>
#include <memory>

namespace curan
{
    namespace ui
    {
        class MutatingTextPanel;
        using text_defined_callback = std::function<void(MutatingTextPanel* button, const std::string& ,ConfigDraw* config)>;

        class MutatingTextPanel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<MutatingTextPanel>
        {
            std::vector<text_defined_callback> panel_callback;
            const std::array<std::string, 3> kTypefaces = {"sans-serif", "serif", "monospace"};
            const size_t kTypefaceCount = kTypefaces.size();
            const std::optional<std::string> default_text;

            static constexpr SkFontStyle::Weight kFontWeight = SkFontStyle::kNormal_Weight;
            static constexpr SkFontStyle::Width kFontWidth = SkFontStyle::kNormal_Width;
            static constexpr SkFontStyle::Slant kFontSlant = SkFontStyle::kUpright_Slant;

            bool has_text_from_user = false;
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
            bool fBlink = false;
            size_t repeated_deleting = 0;
            bool fMouseDown = false;
            size_t counter = 0;
            bool is_highlighted = false;
            bool fTight = false;

            SkColor4f text_color;
            SkColor4f default_text_color;
            SkColor4f background_color;
            SkColor4f highlighted_background_color;
            SkColor4f cursor_color;
            SkColor4f selection_color;

            explicit MutatingTextPanel(bool is_tight);
            explicit MutatingTextPanel(bool is_tight,const std::string& default_text);
        public:
            enum typeface
            {
                sans_serif = 0,
                serif = 1,
                monospace = 2
            };

            template<typename ...T>
            static std::unique_ptr<MutatingTextPanel> make(T&& ...u){
                std::unique_ptr<MutatingTextPanel> editor = std::unique_ptr<MutatingTextPanel>(new MutatingTextPanel(std::forward<T>(u)...));
                return editor;
            }

            void compile() override;

            ~MutatingTextPanel();

            inline void add_textdefined_callback(text_defined_callback callback){
                panel_callback.push_back(callback);
            }

            inline SkRect get_drawable_content(){
                if(fTight){
                    auto val = get_position();
                    SkRect drawable = get_size();
                    drawable.offsetTo(val.centerX() - drawable.width() / 2.0f, val.centerY() - drawable.height() / 2.0f);
                    return drawable;
                } else {
                    return get_position();
                }

            }

            curan::ui::drawablefunction draw() override;

            curan::ui::callablefunction call() override;

            void framebuffer_resize(const SkRect &new_page_size) override;

            void panel_triggered();

            void setFont(typeface font);

            bool moveCursor(SkPlainTextEditor::Editor::Movement m, bool shift = false);

            bool move(SkPlainTextEditor::Editor::TextPosition pos, bool shift);

            void appendtext(const std::string &in_text);

            void replacetext(const std::string &in_text);

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

            inline SkColor4f get_default_text_color()
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                return default_text_color;
            }

            inline MutatingTextPanel &set_default_text_color(SkColor4f color)
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                default_text_color = color;
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


            inline MutatingTextPanel &set_highlighted_color(SkColor4f new_waiting_color)
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                highlighted_background_color = new_waiting_color;
                return *(this);
            }

            inline SkColor4f get_highlighted_state()
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                return highlighted_background_color;
            }
        };

    }
}

#endif