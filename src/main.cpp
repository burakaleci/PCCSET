#include "PCCSET.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PCCSET pccset;
    pccset.show();
    return a.exec();
}
