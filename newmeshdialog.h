#ifndef NEWMESHDIALOG_H
#define NEWMESHDIALOG_H

#include <QDialog>

class Controller;

namespace Ui {
    class NewMeshDialog;
}

class NewMeshDialog : public QDialog
{
    Q_OBJECT

public:
    explicit NewMeshDialog(QWidget *parent = 0);
    ~NewMeshDialog();

    void buildNewMesh(Controller &c);

private:
    Ui::NewMeshDialog *ui;
};

#endif // NEWMESHDIALOG_H
