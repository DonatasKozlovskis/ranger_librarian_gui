/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ranger_librarian_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ranger_librarian_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    ui.view_logging->setModel(qnode.loggingModel());
    // reset image
    ui.image_frame->setImage(QImage());

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(userImageUpdated()), this, SLOT(updateUserView()));
    QObject::connect(&qnode, SIGNAL(navigatorActionStringUpdated()), this, SLOT(updateNavigatorActionString()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());

    /*********************
    ** Auto Start
    **********************/
    if ( !qnode.init() ) {
        showNoMasterMessage();
    } else {
        //connect success
    }

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    closeEvent(0);
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_confirm_clicked(bool check ) {

}

void MainWindow::on_button_reject_clicked(bool check ) {

}

void MainWindow::updateUserView() {
    ui.image_frame->setImage(qnode.q_user_image_);
}

void MainWindow::updateNavigatorActionString() {
    ui.label_navigator->setText(QString(qnode.navigatorActionString().c_str()));
}



void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace ranger_librarian_gui

