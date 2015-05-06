/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/ranger_librarian_gui/main_window.hpp"

static const bool FULLSCREEN = false;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    ranger_librarian_gui::MainWindow w(argc,argv);
    w.show();
    if (FULLSCREEN) {
        w.showFullScreen();
    }

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
