#include "chngdetect.h"
#include "ui_chngdetect.h"
#include "utilIO.hpp"
#include "common.hpp"
#include "pipelines.hpp"
#include "meshProcess.hpp"

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtGui/QFileDialog>
#include <QtCore/QProcess>
#include <QtCore/QTimer>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vtkRenderWindow.h>
#include <boost/thread.hpp>
#include <ostream>
#include <time.h>

pcl::visualization::PCLVisualizer pviz ("test_viz", false);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGBA>);

std::map<int, std::string> input_strings;
pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr view_points (new pcl::PointCloud<pcl::PointXYZ>);

bool model_load = false;
bool change_det = false;
int click_count = 0;
enum detection_technique{POINT_GROUPING, IMG_DIFFERENCE, PSA};
int curr_det_tech = 0;
double resolution = 0.1;
double resolutionVox = 0.1;
double alpha = 1;

chngDetect::chngDetect(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::chngDetect)
{
    input_strings[MESH] = "";
    input_strings[PMVS] = "";
    input_strings[OUTDIR] = "newNVMfile.nvm";
    input_strings[BUNDLER] = "";
    input_strings[CHANGEMASK] = "";

    //CmdIO::callCmd("rm change_mask*");
    ui->setupUi(this);
    ui->label_4->setScaledContents(true);
    ui->label_5->setScaledContents(true);
    ui->label_7->setScaledContents(true);
}

chngDetect::~chngDetect()
{
    delete ui;
}
bool chngDetect::checkInput(){

    if(input_strings[PMVS].compare("")==0){
           //ui->plainTextEdit->appendPlainText("The new image list is missing!");
       return false;
    }
    if(input_strings[BUNDLER].compare("")==0){
           //ui->plainTextEdit->appendPlainText("The old NVM file is missing!");
     return false;
    }

    return true;
}
// Get old PLY file
void chngDetect::on_pushButton_2_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load old model PLY file..."), "", tr("Files (*.ply)"));
    input_strings[MESH] = fileName.toStdString();

        if (fileName != "")
        {
            pviz.removeAllPointClouds();
            pviz.setBackgroundColor(255,255,255);
            ui->qvtkWidget->setDisabled(true);

            pcl::io::loadPLYFile<pcl::PointXYZRGBA> (fileName.toStdString(), *cloud_xyz);            
            ui->qvtkWidget->setEnabled(true);

            if (cloud_xyz->size() > 0)
            {
                pviz.addPointCloud(cloud_xyz,"model");
                ui->qvtkWidget->SetRenderWindow(pviz.getRenderWindow());                
            }
            model_load = true;
        }

        if(change_det){
            ui->pushButton_3->setEnabled(true);
            ui->pushButton_5->setEnabled(true);
            ui->pushButton_11->setEnabled(true);
            ui->lineEdit->setEnabled(true);
            ui->lineEdit_3->setEnabled(true);
        }
        ui->plainTextEdit->appendPlainText(fileName+" loaded");
        pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");
}
//Get NVM file
void chngDetect::on_pushButton_clicked()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Load old model NVM file..."), "", tr("Files (*.nvm)"));
    input_strings[BUNDLER] = file_name.toStdString();
    ui->pushButton_6->setEnabled(checkInput());
    ui->plainTextEdit->appendPlainText(file_name+" loaded");
}
//Show change mask
void chngDetect::on_pushButton_5_clicked()
{
    pviz.removePointCloud("change_mask");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr change_mask(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::io::loadPLYFile<pcl::PointXYZRGBA> ("change_mask.ply", *change_mask);
    ui->qvtkWidget->setEnabled(true);

    if (cloud_xyz->size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(change_mask, 255, 0, 0);
        pviz.addPointCloud(change_mask, single_color,"change_mask");
        pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->verticalSlider->value(), "change_mask");
    }
    ui->qvtkWidget->update();
}

void chngDetect::on_horizontalSlider_sliderMoved(int position)
{
     pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, position, "change_mask");
     ui->qvtkWidget->update();
}
//Change mask size handle
void chngDetect::on_verticalSlider_sliderMoved(int position)
{
    pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, position, "change_mask");
    ui->qvtkWidget->SetRenderWindow(pviz.getRenderWindow());
    ui->qvtkWidget->update();
}

void chngDetect::on_verticalSlider_valueChanged(int value)
{
    on_verticalSlider_sliderMoved(value);
}

void chngDetect::on_verticalSlider_sliderReleased()
{
    on_verticalSlider_sliderMoved(ui->verticalSlider->value());
}
//Run energy minimization
void chngDetect::on_pushButton_3_clicked()
{
    input_strings[MESH] = "old_model.ply";
    input_strings[CHANGEMASK] = "change_mask.ply";
    energyMin(input_strings, resolution, alpha);
    pviz.removePointCloud("change_mask_mrf");

    pcl::PointCloud<pcl::PointXYZ>::Ptr change_mask_mrf(new pcl::PointCloud<pcl::PointXYZ>);
    MeshIO::getPlyFilePCL("change_mask_MRF.ply",change_mask_mrf);   
    ui->qvtkWidget->setEnabled(true);

    if (cloud_xyz->size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(change_mask_mrf, 0, 255, 0);

        pviz.addPointCloud<pcl::PointXYZ>(change_mask_mrf, single_color,"change_mask_mrf");
        pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "change_mask_mrf");        
    }
    ui->qvtkWidget->update();

}
//Get new image file list
void chngDetect::on_pushButton_4_clicked()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.txt)"));
    input_strings[PMVS] = file_name.toStdString();
    ui->pushButton_6->setEnabled(checkInput());
    ui->plainTextEdit->appendPlainText(file_name+" loaded");

}
//Run the change detection algorithm
void chngDetect::on_pushButton_6_clicked()
{
 stringstream tmp_time;

 ui->plainTextEdit->appendPlainText("Running the change detection algorithm...");

 clock_t tStart = clock();

 switch(curr_det_tech){
    case POINT_GROUPING:
        pipelineCorrespondences(input_strings, ui->spinBox_2->value(), camera_cloud, view_points);
        break;
    case IMG_DIFFERENCE:
        {
        if(ui->comboBox->currentIndex()==1 && (input_strings[MESH].compare("")==0))
        {
            ui->plainTextEdit->appendPlainText("Ray shooting requires old model ply file!");
            return;
        }
        pipelineImgDifference(input_strings, ui->spinBox_2->value(), camera_cloud, view_points, ui->comboBox->currentIndex(),resolutionVox );
        break;
        }
    case PSA:
        {
            pipelinePSA(input_strings, ui->spinBox_2->value(), camera_cloud, view_points, ui->comboBox->currentIndex(),resolutionVox );
            break;
        }
    }
 click_count=camera_cloud->points.size()-1;
 ui->pushButton_7->setEnabled(true);

 if(model_load){
     ui->pushButton_3->setEnabled(true);
     ui->pushButton_5->setEnabled(true);
     ui->pushButton_11->setEnabled(true);
     ui->lineEdit->setEnabled(true);
     ui->lineEdit_3->setEnabled(true);
    }

 tmp_time<<(double)(clock() - tStart)/CLOCKS_PER_SEC;

 std::string tmp_string("Time elapsed[s]: "+ tmp_time.str());
 ui->plainTextEdit->appendPlainText(QString::fromStdString(tmp_string));
 change_det = true;
}
// NEXT VIEW HANDLER
void chngDetect::on_pushButton_7_clicked()
{
    std::vector<string> new_image_files;
    std::vector<string> old_image_files;
    std::vector<string> trans_files;

    FileIO::readNewFiles(input_strings[PMVS], new_image_files);
    FileIO::readNewFiles("neighbor_cameras.txt", old_image_files);
    FileIO::readNewFiles("transformation.txt", trans_files);

    trans_files.resize(new_image_files.size());
    QImage new_image;
    QImage old_image;
    QImage trans_image;


    if(click_count<0)
        click_count=camera_cloud->points.size()-1;

    new_image.load(QString::fromStdString(new_image_files[click_count]));
    old_image.load(QString::fromStdString(old_image_files[click_count*ui->spinBox_2->value()]));

    if(trans_files[click_count*ui->spinBox_2->value()].compare("")!=0){
        trans_image.load(QString::fromStdString(trans_files[click_count*ui->spinBox_2->value()]));        
        ui->label_7->setPixmap(QPixmap::fromImage(trans_image));
    }

    ui->label_5->setPixmap(QPixmap::fromImage(new_image));
    ui->label_4->setPixmap(QPixmap::fromImage(old_image));

    pcl::PointXYZ tmp_pt = camera_cloud->points[click_count];
    pviz.setCameraPosition(camera_cloud->points[click_count].x,camera_cloud->points[click_count].y,camera_cloud->points[click_count].z,0,0,0);
    click_count--;
    ui->qvtkWidget->update();
}
// Detection technique change
void chngDetect::on_comboBox_2_currentIndexChanged(int index)
{
    curr_det_tech = index;

    ui->lineEdit_2->setDisabled(true);

    if(curr_det_tech==IMG_DIFFERENCE){
        ui->comboBox->setEnabled(true);
        if(ui->comboBox->currentIndex()==1)
             ui->lineEdit_2->setEnabled(true);
    }
    else
        ui->comboBox->setEnabled(false);
    if(curr_det_tech==PSA)
        ui->plainTextEdit->appendPlainText("THIS OPTION ONLY SAVES IMAGES AND THEIR NEIGHBORS. PSM has to be performed outside of the program(Matlab).");
}
//Remove change mask
void chngDetect::on_pushButton_8_clicked()
{
    pviz.removePointCloud("change_mask");
    ui->qvtkWidget->update();
}
//Remove mrf mask
void chngDetect::on_pushButton_9_clicked()
{
    pviz.removePointCloud("change_mask_mrf");
    ui->qvtkWidget->update();
}
//Change octree resolution
void chngDetect::on_lineEdit_editingFinished()
{
    QString text = ui->lineEdit->text();
    resolution = text.toDouble();
}

void chngDetect::on_comboBox_currentIndexChanged(int index)
{
    if(index==1)
         ui->lineEdit_2->setEnabled(true);
    else
         ui->lineEdit_2->setDisabled(true);
}

void chngDetect::on_lineEdit_2_editingFinished()
{
    QString text = ui->lineEdit_2->text();
    resolutionVox = text.toDouble();
}
//GT generate
void chngDetect::on_pushButton_10_clicked()
{
  std::map<int,std::string> input_map;
  input_map[0] = input_strings[OUTDIR];

  std::cout<<input_map[0]<<endl;

  QString file_name = QFileDialog::getOpenFileName(this, tr("Load new image GT list..."), "", tr("Files (*.txt)"));
  input_map[1] = file_name.toStdString();
  file_name = QFileDialog::getOpenFileName(this, tr("Load old image GT list..."), "", tr("Files (*.txt)"));
  input_map[2] = file_name.toStdString();
  file_name = QFileDialog::getOpenFileName(this, tr("Load index list..."), "", tr("Files (*.txt)"));
  input_map[3] = file_name.toStdString();

  generateGTcloud(input_map, ui->spinBox_2->value(), camera_cloud, view_points, ui->comboBox->currentIndex(), resolutionVox );
  ui->pushButton_11->setEnabled(true);
}
//Show GT cloud
void chngDetect::on_pushButton_11_clicked()
{
    pviz.removePointCloud("gt_cloud");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gt_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::io::loadPLYFile<pcl::PointXYZRGBA> ("gt_cloud.ply", *gt_cloud);

    if (gt_cloud->points.size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(gt_cloud, 0, 0, 255);
        pviz.addPointCloud(gt_cloud, single_color,"gt_cloud");
        pviz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->verticalSlider->value(), "gt_cloud");
    }
    ui->qvtkWidget->update();
}
//Remove GT cloud
void chngDetect::on_pushButton_12_clicked()
{
    pviz.removePointCloud("gt_cloud");
    ui->qvtkWidget->update();
}
//Compute parameters ROC
void chngDetect::on_pushButton_13_clicked()
{
    std::map<std::string,double> param_map;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr change_mask(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gt_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_model(new pcl::PointCloud<pcl::PointXYZRGBA>);


    pcl::io::loadPLYFile<pcl::PointXYZRGBA> ("gt_cloud.ply", *gt_cloud);
    pcl::io::loadPLYFile<pcl::PointXYZRGBA> ("old_model.ply", *old_model);

    pcl::io::loadPLYFile<pcl::PointXYZRGBA> ("change_mask_MRF.ply", *change_mask);

    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud(gt_cloud);

    std::vector<int> pointIdxNKNSearch(2);
    std::vector<float> pointNKNSquaredDistance(2);

    double avg_dist = 0;
    for(int i = 0; i < gt_cloud->points.size(); i++){
        if(kdtree.nearestKSearch (gt_cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
            avg_dist+=pointNKNSquaredDistance[1];
        }
    }

    avg_dist/=gt_cloud->points.size();

    PclProcessing::getROCparameters(gt_cloud, change_mask, param_map, 10*avg_dist, cloud_xyz->points.size());
    ui->plainTextEdit_2->clear();
    ui->plainTextEdit_2->appendPlainText("TP="+QString::number(param_map["TP"]));
    ui->plainTextEdit_2->appendPlainText("FN="+QString::number(param_map["FN"]));
    ui->plainTextEdit_2->appendPlainText("FP"+QString::number(alpha)+"="+QString::number(param_map["FP"])+"/TN;"+"TPR"+QString::number(alpha)+"="+QString::number(param_map["TP"]/(param_map["FN"]+param_map["TP"]))+";");

   // ui->plainTextEdit_2->appendPlainText("TPR"+QString::number(alpha)+"="+QString::number(param_map["TP"]/(param_map["FN"]+param_map["TP"]))+";");
}

void chngDetect::on_lineEdit_3_editingFinished()
{
    QString text = ui->lineEdit_3->text();
    alpha = text.toDouble();
}

void chngDetect::on_pushButton_14_clicked()
{
    std::map<int,std::string> input_map;
    input_map[0] = input_strings[OUTDIR];

    QString file_name = QFileDialog::getOpenFileName(this, tr("Load PSM mask list..."), "", tr("Files (*.txt)"));
    input_map[1] = file_name.toStdString();
    file_name = QFileDialog::getOpenFileName(this, tr("Load index list..."), "", tr("Files (*.txt)"));
    input_map[3] = file_name.toStdString();

    usePSMmasks(input_map, ui->spinBox_2->value(), camera_cloud, view_points, ui->comboBox->currentIndex(), resolutionVox );
    ui->pushButton_7->setEnabled(true);
    ui->pushButton_5->setEnabled(true);
    ui->pushButton_11->setEnabled(true);
}
