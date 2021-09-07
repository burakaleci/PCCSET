#include "ExcelDialog.h"
#include "ui_ExcelDialog.h"

ExcelDialog::ExcelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExcelDialog)
{
    ui->setupUi(this);
}

ExcelDialog::~ExcelDialog()
{
    delete ui;
}

