/**
 * @file /include/ranger_librarian_gui/main_window.hpp
 *
 * @brief Qt based gui for ranger_librarian_gui.
 *
 * @date November 2010
 **/
#ifndef ranger_librarian_gui_MAIN_WINDOW_H
#define ranger_librarian_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ranger_librarian_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();


public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_button_confirm_clicked(bool check );
    void on_button_reject_clicked(bool check );


    /******************************************
    ** Manual connections
    *******************************************/
    void updateUserView();
    void updateNavigatorActionString();
private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace ranger_librarian_gui

#endif // ranger_librarian_gui_MAIN_WINDOW_H
