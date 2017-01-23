#ifndef OPTDIALOG
#define OPTDIALOG
#include <QDialog>
#include "Properties.h"
#include "util.h"

class OptDialog : public QDialog
{
public:
    OptDialog(Util::Properties *prop);

};
#endif // OPTDIALOG

