#ifndef CHNGDETECT_H
#define CHNGDETECT_H

#include <QtGui/QWidget>
#include <string>

namespace Ui {
class chngDetect;
}

class chngDetect : public QWidget
{
    Q_OBJECT
    
public:
    explicit chngDetect(QWidget *parent = 0);
    ~chngDetect();
    void printMsg(std::string);
    bool checkInput();

private slots:
    void on_pushButton_2_clicked();

    void on_pushButton_clicked();

    void on_pushButton_5_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_verticalSlider_sliderMoved(int position);

    void on_verticalSlider_valueChanged(int value);

    void on_verticalSlider_sliderReleased();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_comboBox_2_currentIndexChanged(int index);

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_lineEdit_editingFinished();

    void on_comboBox_currentIndexChanged(int index);

    void on_lineEdit_2_editingFinished();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();

    void on_pushButton_13_clicked();

    void on_lineEdit_3_editingFinished();

    void on_pushButton_14_clicked();

private:
    Ui::chngDetect *ui;
};

#endif // CHNGDETECT_H
