#include "optdialog.h"
#include <QFormLayout>
#include <QLabel>
#include <QList>
#include <QString>
#include <QLineEdit>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDebug>

OptDialog::OptDialog(Util::Properties * prop)
{
    QDialog dialog(this);
    // Use a layout allowing to have a label next to each field
    QFormLayout form(&dialog);

    // Add some text above the fields
    form.addRow(new QLabel("Options for execution of Harris 3D"));

    // Add the lineEdits with their respective labels
    QList<QLineEdit *> fields;

    QLineEdit *lineEditTypeNeighborhood = new QLineEdit(&dialog);
    lineEditTypeNeighborhood->setText("spatial");
    QString labelTypeNeighborhood = QString("Type neighborhood");
    form.addRow(labelTypeNeighborhood, lineEditTypeNeighborhood);

    fields << lineEditTypeNeighborhood;

    QLineEdit *lineEditParameterNeighborhood = new QLineEdit(&dialog);
    lineEditParameterNeighborhood->setText("0.01");
    QString labelParameterNeighborhoodd = QString("Parameter neighborhood");
    form.addRow(labelParameterNeighborhoodd, lineEditParameterNeighborhood);

    fields << lineEditParameterNeighborhood;

    QLineEdit *lineEditK= new QLineEdit(&dialog);
    lineEditK->setText("0.04");
    QString labelK = QString("K");
    form.addRow(labelK, lineEditK);

    fields << lineEditK;

    QLineEdit *lineEditRingMaxima = new QLineEdit(&dialog);
    lineEditRingMaxima->setText("1");
    QString labelRingMaxima = QString("Ring maxima detection");
    form.addRow(labelRingMaxima, lineEditRingMaxima);

    fields << lineEditRingMaxima;

    QLineEdit *lineEditInterestPoints = new QLineEdit(&dialog);
    lineEditInterestPoints->setText("fraction");
    QString labelInterestPoints = QString("Interest points selection");
    form.addRow(labelInterestPoints, lineEditInterestPoints);

    fields << lineEditInterestPoints;

    QLineEdit *lineEditParameterSelection = new QLineEdit(&dialog);
    lineEditParameterSelection->setText("0.05");
    QString labelParameterSelection = QString("Parameter selection");
    form.addRow(labelParameterSelection, lineEditParameterSelection);

    fields << lineEditParameterSelection;

    QLineEdit *lineEditFilteringSteps = new QLineEdit(&dialog);
    lineEditFilteringSteps->setText("0");
    QString labelFilteringSteps = QString("Filtering steps");
    form.addRow(labelFilteringSteps, lineEditFilteringSteps);

    fields << lineEditFilteringSteps;

    QLineEdit *lineEditAlpha = new QLineEdit(&dialog);
    lineEditAlpha ->setText("0.5");
    QString labelAlpha = QString("Alpha");
    form.addRow(labelAlpha, lineEditAlpha);

    fields << lineEditAlpha;

    QLineEdit *lineEditRotation = new QLineEdit(&dialog);
    lineEditRotation ->setText("4");
    QString labelRotation = QString("Rotations");
    form.addRow(labelRotation, lineEditRotation);

    fields << lineEditAlpha;

    QCheckBox *checkBoxDecimation = new QCheckBox(&dialog);
    checkBoxDecimation->setChecked(true);
    QString labelDecimation = QString("Decimate mesh?");
    form.addRow(labelDecimation, checkBoxDecimation);

    QCheckBox *checkBoxSmoothing = new QCheckBox(&dialog);
    checkBoxSmoothing->setChecked(true);
    QString labelSmoothing = QString("Smoothen surface?");
    form.addRow(labelSmoothing, checkBoxSmoothing);

    QCheckBox *checkBoxKeypoints = new QCheckBox(&dialog);
    checkBoxKeypoints->setChecked(true);
    QString labelKeypoints = QString("Show keypoints?");
    form.addRow(labelKeypoints, checkBoxKeypoints);

    QCheckBox *checkBoxAlignment = new QCheckBox(&dialog);
    checkBoxAlignment->setChecked(true);
    QString labelAlignment = QString("Perform alignment?");
    form.addRow(labelAlignment, checkBoxAlignment);

    QCheckBox *checkBoxSegmentation = new QCheckBox(&dialog);
    checkBoxSegmentation->setChecked(false);
    QString labelSegmentation = QString("Segmentate mesh?");
    form.addRow(labelSegmentation, checkBoxSegmentation);

    QCheckBox *checkBoxHoleFilling = new QCheckBox(&dialog);
    checkBoxHoleFilling->setChecked(false);
    QString labelHoleFilling = QString("Fill holes?");
    form.addRow(labelHoleFilling, checkBoxHoleFilling);

    // Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok,
                               Qt::Horizontal, &dialog);
    form.addRow(&buttonBox);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));

    // Show the dialog as modal
    if (dialog.exec() == QDialog::Accepted) {
        // If the user didn't dismiss the dialog, add property
        std::string str;
        str = lineEditTypeNeighborhood->text().toStdString();
        prop->setProperty("type-neighborhood",prop->trim(str));
        str = lineEditParameterNeighborhood->text().toStdString();
        prop->setProperty("parameter-neighborhood",prop->trim(str));
        str = lineEditK->text().toStdString();
        prop->setProperty("K",prop->trim(str));
        str = lineEditRingMaxima->text().toStdString();
        prop->setProperty("ring-maxima-detection",prop->trim(str));
        str = lineEditInterestPoints->text().toStdString();
        prop->setProperty("interest-points-selection",prop->trim(str));
        str = lineEditParameterSelection->text().toStdString();
        prop->setProperty("parameter-selection",prop->trim(str));
        str = lineEditFilteringSteps->text().toStdString();
        prop->setProperty("filtering-steps",prop->trim(str));
        str = lineEditAlpha->text().toStdString();
        prop->setProperty("alpha",prop->trim(str));
        str = lineEditRotation->text().toStdString();
        prop->setProperty("rotation",prop->trim(str));
        if (checkBoxDecimation->isChecked()){
            prop->setProperty("decimate","1");
        }
        else{
            prop->setProperty("decimate","0");
        }
        if (checkBoxSmoothing->isChecked()){
            prop->setProperty("openvdb","1");
        }
        else{
            prop->setProperty("openvdb","0");
        }
        if (checkBoxKeypoints->isChecked()){
            prop->setProperty("keypoints","1");
        }
        else{
            prop->setProperty("keypoints","0");
        }
        if (checkBoxAlignment->isChecked()){
            prop->setProperty("alignment","1");
        }
        else{
            prop->setProperty("alignment","0");
        }
        if (checkBoxSegmentation->isChecked()){
            prop->setProperty("segmentation","1");
        }
        else{
            prop->setProperty("segmentation","0");
        }
        if (checkBoxHoleFilling->isChecked()){
            prop->setProperty("hole-filling","1");
        }
        else{
            prop->setProperty("hole-filling","0");
        }
    }
}
