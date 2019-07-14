/**
 * @file button_property.h
 * @brief button property
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_RVIZ_PROPERTY_BUTTON_PROPERTY_H
#define TESSERACT_RVIZ_PROPERTY_BUTTON_PROPERTY_H

#include <QStringList>
#include <QPushButton>

#include "rviz/properties/property.h"

namespace tesseract_rviz
{
class ButtonProperty : public rviz::Property
{
  Q_OBJECT
public:
  ButtonProperty(const QString& name = QString(),
                 const QString& default_value = QString(),
                 const QString& description = QString(),
                 rviz::Property* parent = nullptr,
                 const char* changed_slot = nullptr,
                 QObject* receiver = nullptr);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override;

  bool paint(QPainter* painter, const QStyleOptionViewItem& option) const override;

public Q_SLOTS:
  virtual void setCaptions(const QString& str);

protected:
  QPushButton* button_;
  QString captions_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_PROPERTY_BUTTON_PROPERTY_H
