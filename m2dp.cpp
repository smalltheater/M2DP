//
// Created by yunqi on 10/01/2022.
//

#include "include/kitti2pcd.h"

// STL header
#include <fstream>

#include <time.h>

using namespace std;


double my_atan2(double a, double b) { return std::atan2(a,b); }

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud

    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    points->clear();
    vec_x.clear();
    vec_y.clear();
    vec_z.clear();

    i=0;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    //int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        vec_x.push_back(point.x);
        vec_y.push_back(point.y);
        vec_z.push_back(point.z);
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}

Eigen::MatrixXd PCARotationInvariant(pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{

 //cout<<"There are "<<n<<"points."<<endl;
 double mean_x,mean_y,mean_z;
 double std_x,std_y,std_z;
 pcl::getMeanStd(vec_x, mean_x, std_x);
 pcl::getMeanStd(vec_y, mean_y, std_y);
 pcl::getMeanStd(vec_z, mean_z, std_z);


 Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
 transform_matrix.translation()<<mean_x,mean_y,mean_z;
// cout << transform_matrix.matrix() << endl;

 pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZI>);
 pcl::transformPointCloud(*points,*transform_cloud,transform_matrix);


// Visualization
/***
 pcl::visualization::PCLVisualizer viewer("矩阵转换实例");
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> points_clolor(points,255,255,255);
 viewer.addPointCloud(points,points_clolor,"original_color");

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_color(transform_cloud,250,0,0);
viewer.addPointCloud(transform_cloud,transformed_cloud_color,"transformed_cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce()
                ;}

***/

    pcl::PointCloud<pcl::PointXYZI>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCA<pcl::PointXYZI> pcaGolden;
    pcaGolden.setInputCloud(transform_cloud);
    EigenDataAfterPCA = pcaGolden.getCoefficients();
    EigenDataAfterPCA.row(2)=EigenDataAfterPCA.row(2)*-1;
   // EigenDataAfterPCA=EigenDataAfterPCA.transpose();

    Eigen::MatrixXd data=EigenDataAfterPCA.cast<double>();

    return data;


/**  save data
  ofstream file("data.txt");
    if(file.is_open())
    {
        file << "Here is the matrix m:\n" << goldenEVs_Dir.transpose() << '\n';
        file << "m" << '\n' ;
    }
*/

}

double getMaxRho(Eigen::MatrixXd TempMatrix)
{
    Eigen::MatrixXd use=TempMatrix.transpose();
    double maxRho=0;
    double temp;
    for(int i=0;i<use.rows();i++)
    {
        temp=use.row(i).squaredNorm();
        if(temp>maxRho)
            maxRho=temp;
    }

     maxRho= sqrt(maxRho);


    return maxRho;

}

Eigen::VectorXd buildVector(double start, double end,int steps)
{
   Eigen::VectorXd G_temp = Eigen::VectorXd::LinSpaced(steps,start,end);
    return G_temp;

}

Eigen::VectorX<long double> CountVote2D(Eigen::VectorXd theta,Eigen::VectorXd thetaLIst,Eigen::VectorXd rho,Eigen::VectorXd rhoList)
{
    int i,n;
    int idxR, idxT;
    Eigen::VectorX<long double> pVoteCnt;
    pVoteCnt=Eigen::VectorX<long double>::Zero(numR*numT);

    for(n=0;n<theta.size();n++)
    {

        idxT=numT-1;
        for(i=0;i<numT-1;i++)
        {
            if(theta[n]<thetaLIst[i+1])
            {
                idxT=i;
                break;
            }
        }
        idxR=numR-1;
        for(i=0;i<numR-1;i++)
        {
            if(rho[n]<rhoList[i+1])
            {
                idxR=i;
                break;
            }
        }
        (pVoteCnt(idxR*numT+idxT))++;
    }


/*
    for(int t=0;t<numT;t++)
    {
        for (int r = 0; r < numR; r++)
        {

            cout << pVoteCnt(r * numT + t) << "  ";
        }
        cout<<endl;
    }
*/
    return pVoteCnt;

}



Eigen::MatrixXd GetSignatureMatrix(Eigen::VectorXd azimuthList,Eigen::VectorXd elevationList,Eigen::MatrixXd data,int numT,int numR,double maxRho)
{
    Eigen::Vector3d temp(1,0,0);


   int n=0;
   Eigen::MatrixXd A= Eigen::MatrixXd::Zero(azimuthList.size()*elevationList.size(),numT*numR);
   Eigen::VectorXd thetaLIst= buildVector(-M_PI,M_PI,numT+1);

   Eigen::VectorXd rhoListTemp= buildVector(0, sqrt(maxRho),numR+1);
   Eigen::VectorXd rhoList=rhoListTemp.array().square();

   double azm;
    double elv;
    Eigen::Vector3d vecN;
    double h;
    Eigen::Vector3d c;
    Eigen::Vector3d px;
    Eigen::Vector3d py;
    Eigen::VectorXd rho;
    Eigen::MatrixXd MatrixX;
    Eigen::MatrixXd MatrixY;
    Eigen::VectorXd theta;
    Eigen::VectorX<long double> bin;
for(int p=0;p<azimuthList.size();p++)
{
    azm=azimuthList(p);  //Yaw angle  0-360  in this paper is (-90 ~ +90)
    for(int q=0;q<elevationList.size();q++)
    {
        elv=elevationList(q);    // Pitch angle 0-90

        vecN<< cos(azm)* cos(elv),sin(azm)*cos(elv),sin(elv);


        h=temp.dot(vecN);

        c=h*vecN;
        px=temp-c;
        py=vecN.cross(px);

        rho=((data*px).array().square()+(data*py).array().square()).sqrt();
        MatrixX=data*px;
        MatrixY=data*py;


        theta=MatrixY.binaryExpr(MatrixX,std::ptr_fun(::my_atan2));


        bin=CountVote2D( theta, thetaLIst, rho,rhoList)/theta.size();


        A.row(n)=bin.transpose().cast<double>();

        n++;
    }

}

return A;
}


int main(int argc, char **argv)
{
    if(argc<3)
    {
        cout<<"It's need an input file and an output file"<<endl;
        return 0;
    }
    cout<<"Input file name: " <<argv[1]<< endl;
    cout<<"Output file name: "<<argv[2]<< endl;
    string InputFile    =argv[1];
    string OutputFile   =argv[2];

    readKittiPclBinData(InputFile,OutputFile);

    clock_t start,end,start_a;//
    start=clock();//
    start_a=clock();

//To do





    Eigen::MatrixXd data = PCARotationInvariant(points);

    end=clock();//
    printf("\n\nRunning Time1：%dms\n", (end-start)/1000);
    start=clock();//


    double maxRho=getMaxRho(data);
    end=clock();//
    printf("\n\nRunning Time2：%dms\n", (end-start)/1000);//


    Eigen::VectorXd azimuthList= buildVector(-M_PI/2,M_PI/2,numP);
    Eigen::VectorXd elevationList= buildVector(0,M_PI/2,numQ);

    Eigen::MatrixXd PCAData=data.cast<double>();
    start=clock();
    Eigen::MatrixXd A=GetSignatureMatrix( azimuthList, elevationList, PCAData.transpose(), numT, numR, maxRho);
    end=clock();
    printf("\n\nRunning Time3：%dms\n", (end-start)/1000);//


start=clock();

    Eigen::BDCSVD<Eigen::MatrixXd> svd( A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();

    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd res;
    res.resize(U.rows()+V.rows());
    res<<U.col(0),
    V.col(0);

    end=clock();
    printf("\n\nRunning Time4：%dms\n", (end-start)/1000);
    printf("\n\nRunning Time4：%dms\n", (end-start_a)/1000);


    return 0;
}