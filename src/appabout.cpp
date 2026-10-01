/*  OpenFIRE App: a configuration utility for the OpenFIRE light gun system.
    App info window.

    Copyright (C) 2025  Team OpenFIRE

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "appabout.h"
#include "ui_appabout.h"

#include <QDesktopServices>
#include <QUrl>

AppAbout::AppAbout(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::AppAbout)
{
    ui->setupUi(this);

    // Keep the designed width, but grow the height if the texts need it
    // (longer translations, larger system fonts).
    int h = this->height();
    if(this->layout()->hasHeightForWidth())
        h = qMax(h, this->layout()->totalHeightForWidth(this->width()));
    this->setFixedSize(this->width(), h);
    this->setWindowFlags(Qt::MSWindowsFixedSizeDialogHint | Qt::WindowCloseButtonHint);

    ui->topText->setText(ui->topText->text() +
#ifdef QT_VERSION_MAJOR
                         QString::number(QT_VERSION_MAJOR) + '.' +
#ifdef QT_VERSION_MINOR
                         QString::number(QT_VERSION_MINOR) + '.' +
#ifdef QT_VERSION_PATCH
                         QString::number(QT_VERSION_PATCH)
#endif
#endif
#endif
                         );

    connect(ui->kofiButton, &QPushButton::clicked, this, []() {
        QDesktopServices::openUrl(QUrl("https://ko-fi.com/alessandrosatanassi"));
    });
}

AppAbout::~AppAbout()
{
    delete ui;
}
