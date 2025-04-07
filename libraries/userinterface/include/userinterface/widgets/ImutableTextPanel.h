#ifndef CURAN_IMUTABLE_TEXTPANEL_HEADER_FILE_
#define CURAN_IMUTABLE_TEXTPANEL_HEADER_FILE_

#include "userinterface/widgets/ConfigDraw.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "include/core/SkFontMgr.h"
#include "skplaintexteditor/editor.h"

#include <cfloat>
#include <fstream>
#include <iostream>
#include <memory>

namespace curan{
namespace ui{

/*

*/

class ImutableTextPanel final : public curan::ui::Drawable, public curan::utilities::Lockable
{
public:

    enum typeface{
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

    inline SkColor4f get_text_color(){
        std::lock_guard<std::mutex> g{get_mutex()};
        return text_color;
    }

    inline ImutableTextPanel &set_text_color(SkColor4f color){
        std::lock_guard<std::mutex> g{get_mutex()};
        text_color = color;
        return *(this);
    }

    inline SkColor4f get_background_color(){
        std::lock_guard<std::mutex> g{get_mutex()};
        return background_color;
    }

    inline ImutableTextPanel &set_background_color(SkColor4f color){
        std::lock_guard<std::mutex> g{get_mutex()};
        background_color = color;
        return *(this);
    }

    inline ImutableTextPanel &text(const std::string& other){
        std::lock_guard<std::mutex> g{get_mutex()};
        SkPlainTextEditor::Editor::TextPosition final_position{fEditor.lineCount() - 1, fEditor.line(fEditor.lineCount() - 1).size};
        fEditor.remove(SkPlainTextEditor::Editor::TextPosition{0, 0}, final_position);
        fEditor.insert(SkPlainTextEditor::Editor::TextPosition{0, 0}, other.data(), other.size());
        return *(this);
    }

    inline ImutableTextPanel &appendtext(const std::string& other){
        std::lock_guard<std::mutex> g{get_mutex()};
        SkPlainTextEditor::Editor::TextPosition final_position{0, index};
        index = (index+1) % 30;
        fEditor.insert(final_position, other.data(), other.size());
        return *(this);
    }

private:

    ImutableTextPanel(const std::string& default_text);

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

    size_t index = 0;

};

}
}

#endif