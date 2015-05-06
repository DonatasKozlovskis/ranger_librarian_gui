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

    // reset image on topic change
    ui.image_frame->setImage(QImage());

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(rgbImageUpdated()), this, SLOT(updateUserView()));

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
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {

//}

void MainWindow::updateUserView() {
    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation

    QImage image( qnode.conversion_mat_.data, qnode.conversion_mat_.cols, qnode.conversion_mat_.rows, qnode.conversion_mat_.step[0], QImage::Format_RGB888);

    // reset image on topic change
    ui.image_frame->setImage(image);

//    cv::waitKey(3);

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace ranger_librarian_gui

