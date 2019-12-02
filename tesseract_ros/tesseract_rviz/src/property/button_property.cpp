#include <tesseract_rviz/property/button_property.h>
#include <QPainter>
#include <QStyleOptionViewItem>

namespace tesseract_rviz
{
ButtonProperty::ButtonProperty(const QString& name,
                               const QString& default_value,
                               const QString& description,
                               rviz::Property* parent,
                               const char* changed_slot,
                               QObject* receiver)
  : Property(name, default_value, description, parent, changed_slot, receiver), button_(nullptr), captions_("")
{
}

void ButtonProperty::setCaptions(const QString& str)
{
  captions_ = str;
  if (button_)
    button_->setText(str);
}

bool ButtonProperty::paint(QPainter* painter, const QStyleOptionViewItem& option) const
{
  painter->save();
  QTextOption text_option;
  text_option.setAlignment(Qt::AlignmentFlag::AlignCenter);
  QRect rect = option.rect;
  //  rect.adjust( rect.height() + 4, 1, 0, 0 );
  painter->drawText(rect, captions_, text_option);
  painter->restore();

  return true;  // return true, since this function has done the painting.
}

QWidget* ButtonProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& /*option*/)
{
  button_ = new QPushButton(parent);
  button_->setText(captions_);
  QObject::connect(button_, SIGNAL(clicked()), this, SIGNAL(changed()));

  return button_;
}

}  // namespace tesseract_rviz
