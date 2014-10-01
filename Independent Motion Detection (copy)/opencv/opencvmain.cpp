
#include <yarp/os/RFModule.h>
// #include <yarp/os/Module.h>
// #include <yarp/os/Network.h>
// #include <yarp/sig/Vector.h>
#include "gurls++/gurls.h"
// #include "gurls++/exceptions.h"
// #include "gurls++/gmat2d.h"
// #include "gurls++/options.h"
// #include "gurls++/optlist.h"
// #include "gurls++/gmath.h"
// #include <string>
// #include <yarp/os/RateThread.h>
// #include <yarp/os/BufferedPort.h>
// #include <yarp/os/Time.h>
// #include <yarp/os/Semaphore.h>
#include <yarp/dev/all.h>
// #include <../src/boost/libs/date_time/test/testfrmwk.hpp>
#include <cv.h>
// #include <highgui.h>
// #include <iostream>
// #include <algorithm>
// #include <stdio.h>
// #include <stdlib.h>
// #include <cstdio>
// #include <vector>
// #include <fstream>
// #include <sstream>
// #include <iomanip>
// #include <math.h>
#include <ml.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace cv;
using namespace gurls;

typedef double T;
class MLModule
{
    int velocityparams;
    int trainingdata[101][1];
    int trainingdatasize;
    double label1[101];
    double label2[101];
    double label3[101];
    double label4[101];
    double label5[101];
    CvSVM* svm1;
    CvSVM* svm2;
    CvSVM* svm3;
    CvSVM* svm4;
    CvSVM* svm5;
public:
    void trainData(){
        velocityparams = 1; 
        readDataFromFile();
        cout<<"read from file";
        cout.flush();
        svm1 = performMLModel ( label1 );
        svm2 = performMLModel ( label2 );
        svm3 = performMLModel ( label3 );
        svm4 = performMLModel ( label4 );
        svm5 = performMLModel ( label5 );
    }
    void readDataFromFile()
    {
        trainingdatasize =  101;
        ifstream file ;
        file.open ( "..//..//opencv//build//toLearn1.csv" );
        cout<<" reading data from file "<< file.good();
        std::string line;
        int linenumber = 0 ;
        while ( std::getline ( file,line ) )
        {
            std::stringstream  lineStream ( line );
            std::string        cellvalue;
            int cellnumber = 0;
            while ( std::getline ( lineStream,cellvalue,',' ) )
            {
                double cellvalued = atof ( cellvalue.c_str() );
		cout<<cellvalue<<" "<<cellvalued <<"    ";
                if ( cellnumber < velocityparams )
                {
		    
                    trainingdata[linenumber][cellnumber] = cellvalued;
                }
                else
                {

                    switch ( cellnumber )
                    {

                    case 1:
                        label1[linenumber] = cellvalued;
                        break;

                    case 2:
                        label2[linenumber] = cellvalued;
                        break;
                    case 3:
                        label3[linenumber] = cellvalued;
                        break;
                    case 4:
                        label4[linenumber] = cellvalued;
                        break;
                    case 5:
                        label5[linenumber] = cellvalued;
                        break;
                    default:
                        cout<< "problem";
                    }


                }
                cellnumber++;
            }
            cout<<endl;
            linenumber++;
        }
        cout.flush();
        file.close();

    }

    CvSVM* performMLModel ( double label[] )
    {
        Mat labelsMat ( 101, 1, CV_32FC1, label );
        Mat trainingDataMat ( 101, 1, CV_32FC1, trainingdata );
        // Set up SVM's parameters
        CvSVMParams params;
        params.svm_type    = CvSVM::NU_SVR;
        params.kernel_type = CvSVM::LINEAR;
        params.nu = 0.5;
        params.term_crit   = cvTermCriteria ( CV_TERMCRIT_ITER, 10, 1e-2 );
        // Train the SVM
        CvSVM *SVM = new CvSVM;
	SVM->train ( trainingDataMat, labelsMat, Mat (), Mat(), params );
        int c  = SVM->get_support_vector_count();
        for ( int i = 0; i < c; ++i )
        {
            const float* v = SVM->get_support_vector ( i );
            cout<<" support vector i "<< ( int ) v[0] <<" , "<< ( int ) v[1] <<" , "<< ( int ) v[2] <<" , "<< ( int ) v[3] <<" , "<< ( int ) v[4]  <<endl;
        }
        return SVM;
    }



  

    double* predictML(double velocityArray)
    {
	double sz[1] = {velocityArray};
        Mat sampleMat ( 1, 1, CV_32FC1, sz );
	double mupredict = svm1->predict ( sampleMat );
	double mvpredict = svm2->predict ( sampleMat );
	double siguupredict = svm3->predict ( sampleMat );
	double sigvvpredict = svm4->predict ( sampleMat );
	double siguvpredict = svm5->predict ( sampleMat );
	double *statistics = new double[5];
        statistics[0]= mupredict;
	statistics[1]= mvpredict;
	statistics[2]= siguupredict;
	statistics[3]= sigvvpredict;
	statistics[4]= siguvpredict;
	return statistics;
    }
};




class ML{
      GURLS G;
      GurlsOptionsList* opt;
      OptTaskSequence *seq;
      GurlsOptionsList * process;
      OptString* hofun;
      OptFunction* optfun;
public:
  void trainData(){
    gMat2D<T> ML_velocityArray, ML_flowstatistics_train;//, ML_mv_train, ML_siguu_train,ML_sigvv_train,ML_siguv_train;
    ML_velocityArray.readCSV("velocityArray.csv");
    ML_flowstatistics_train.readCSV("flowstatistics_train.csv");
    opt = new GurlsOptionsList("IndependentMotionDetection", true);
    seq = new OptTaskSequence();
    *seq << "split:ho" << "paramsel:hoprimal" << "optimizer:rlsprimal";
    *seq << "pred:primal" << "perf:rmse";
    opt->addOpt("seq", seq);
    process = new GurlsOptionsList("processes", false);
    OptProcess* process1 = new OptProcess();
    *process1 << GURLS::computeNsave << GURLS::computeNsave << GURLS::computeNsave;
    *process1 << GURLS::ignore << GURLS::ignore;
    process->addOpt("one", process1);
    OptProcess* process2 = new OptProcess();
    *process2 << GURLS::load << GURLS::load << GURLS::load;
    *process2 << GURLS:: computeNsave << GURLS:: ignore;
    process->addOpt("two", process2);
    opt->addOpt("processes", process);
    hofun = new OptString("rmse");
    opt->removeOpt("hoperf");
    opt->addOpt("hoperf", hofun);
    optfun = new OptFunction("mean");
    opt->removeOpt("singlelambda");
    opt->addOpt("singlelambda",optfun);
    string jobId0("one");
    G.run(ML_velocityArray, ML_flowstatistics_train, *opt, jobId0); 
  }

  gMat2D<T> predictML(double velocityArray){
       cout<< "one"<< endl;
    gMat2D<T> ML_velocityArray_test(1,1);
    gMat2D<T> ML_flowstatistics_test(1,5);
    ML_velocityArray_test=velocityArray;
    ML_flowstatistics_test=0;
    string jobId1("two");
    G.run(ML_velocityArray_test, ML_flowstatistics_test, *opt, jobId1);
    const gMat2D<T>& pred_mat = OptMatrix<gMat2D<T> >::dynacast(opt->getOpt("pred"))->getValue();
    return pred_mat;
  }
};

class Utilities {
      public :
	double findSig ( int j, double fu,double fv, std::vector<cv::Point2f> cornersL1Local,std::vector<cv::Point2f> cornersL2Local, std::vector<uchar> features_found )
    {
        double u,sigu;
	double i2x , i2y , i1x , i1y ;
	u = 0;
        for ( int i=0; i < features_found.size(); i++ )
        {
	   i2x = (double)cornersL2Local[i].x;
	   i2y = (double)cornersL2Local[i].y;
	   i1x = (double)cornersL1Local[i].x;
	   i1y = (double)cornersL1Local[i].y;
	    if ( j == 1 )
                u = u + pow ( ( i2x - i1x ) - fu, 2.0 );// * ( ( cornersL2[i].x - cornersL1[i].x ) - mu ) ;
            if ( j == 2 )
                u = u + pow ( ( i2y - i1y ) - fv, 2.0 );// * ( ( cornersL2[i].y - cornersL1[i].y ) - mv ) ;
            if ( j == 3 )
                u = u + pow ( ( i2x - i1x ) - fv, 2.0 );// * ( ( cornersL2[i].y - cornersL1[i].y ) - mv ) ;
            if ( j == 4 )
                u = u + pow ( ( i2y - i1y ) - fu, 2.0 );// * ( ( cornersL2[i].x - cornersL1[i].x ) - mu ) ;
        }
        sigu = u/features_found.size();
	return sigu;
    }
    
        double findMu ( int j,std::vector<cv::Point2f> cornersL1Local,std::vector<cv::Point2f> cornersL2Local, std::vector<uchar> features_found )
    {
        double u,mu;
	double i2x , i2y , i1x , i1y ;
	u = 0;
        for ( int i=0; i < features_found.size(); i++ )
        {
	   i2x = (double)cornersL2Local[i].x;
	   i2y = (double)cornersL2Local[i].y;
	   i1x = (double)cornersL1Local[i].x;
	   i1y = (double)cornersL1Local[i].y;
            if ( j == 1 )
                u += ( i2x - i1x);
            if ( j == 2 )
                u += ( i2y - i1y );
        }
        mu = u/features_found.size();
        return mu;
    }
};


class MyModule:public RFModule
{
    BufferedPort<ImageOf<PixelRgb>  >  imageLIn;
    BufferedPort<ImageOf<PixelRgb>  >  imageRIn;
    BufferedPort<ImageOf<PixelRgb>  >  imageLOut;
    BufferedPort<ImageOf<PixelRgb>  >  imageROut;
    BufferedPort<Bottle>          port_rpc_human;
    Port port;
    String strhuman_cmd;
    cv::Mat src_gray;
    double bottlevector1;
    double bottlevector2;
    double velocityArray;
    cv::Mat imageL1;
    cv::Mat imageL2;
    cv::Mat imageR1;
    cv::Mat imageR2;
    int lock;
    int trainingdatasize;
    double mu,mv,siguu,siguv,sigvv,sigvu;
    ofstream file;
    Property optGaze1;
    PolyDriver      robotHead1;
    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IEncodersTimedRaw *encTimed;
    double position;
    bool Flag;
    int jnts;
    Utilities t;
    ML ml;
    double threshold;
    int hm_check;
    Bottle head_move_check;
    int total_anom_count;
    int total_frames_eval;
    int anom_counter;
    int error_counter;
    int check_predict_type;
public:
    double getPeriod()
    {
        return 0.1;
    }
    cv::Mat blueFilter ( const cv::Mat& src )
    {
        assert ( src.type() == CV_8UC3 );
        cv::Mat blueOnly;
        cv::inRange ( src, cv::Scalar ( 0, 0, 0 ), cv::Scalar ( 0, 0, 255 ), blueOnly );
        return blueOnly;
    }

    double returnVelocity()
    {
      enc->getEncoder(4, &position);
      return position;
    }

    void imageAcq()
    {
        if ( imageL1.data == NULL and imageR1.data == NULL )
        {
            imageL1 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR1 = ( IplImage * ) imageRIn.read()->getIplImage();
            bottlevector1 = returnVelocity();
            imageL2 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR2 = ( IplImage * ) imageRIn.read()->getIplImage();
            bottlevector2 = returnVelocity();
        }
        else
        {
            imageL1 = imageL2;
            imageR1 = imageR2;
            bottlevector1 = bottlevector2;
            imageL2 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR2 = ( IplImage * ) imageRIn.read()->getIplImage();
            bottlevector2 = returnVelocity();
        }
    }

    void findDiffVelocity()
    {
        if ( bottlevector1 != 0 and bottlevector2 != 0 )
        {
	  velocityArray = bottlevector2 - bottlevector1;
        }
    }

    cv::Point findBall(Mat mat_left){
      cv::Mat blueOnly = blueFilter(mat_left);
            float sumx=0, sumy=0;
            float num_pixel = 0;
            for(int x=0; x<blueOnly.cols; x++) {
                for(int y=0; y<blueOnly.rows; y++) {
                    int val = blueOnly.at<uchar>(y,x);
                    if( val >= 50) {
                        sumx += x;
                        sumy += y;
                        num_pixel++;
                    }
                }
            }
            cv::Point p(sumx/num_pixel, sumy/num_pixel);
	    return p;
    }

    bool updateModule()
    {
        if ( imageLIn.getInputCount() >0 )
        {
            // ****************** Image Acquisition ***************************
            imageAcq();
            Mat tempimageL1, tempimageL2;
	    imageL1.copyTo(tempimageL1);
	    imageL1.copyTo(tempimageL2);
            findDiffVelocity();
	    cv::Point p = findBall(tempimageL1);
            ImageOf<PixelRgb> &outLImg = imageLOut.prepare();
            ImageOf<PixelRgb> &outRImg = imageROut.prepare();
            // *************** Obtain feature points ************************  Feature extracting
            std::vector<cv::Point2f> cornersL2 = harrisCorner2 ( imageL2 );
            // ***************************************** Tracking ***************************
            std::vector<uchar> features_found;
            std::vector<float> feature_errors;
            std::vector<cv::Point2f> cornersL1;
            cv::Mat imageL11;
            cv::Mat imageL22;
            cv::cvtColor ( imageL2, imageL22, CV_BGR2GRAY );
            cv::cvtColor ( tempimageL1, imageL11, CV_BGR2GRAY );
            calcOpticalFlowPyrLK ( imageL22, imageL11, cornersL2, cornersL1, features_found, feature_errors ,Size ( 15, 15 ), 5,
                                   cvTermCriteria ( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );
            // to display output for this harriscorner2
            for ( int i=0; i < cornersL2.size(); i++ )
            {
                circle ( tempimageL1, Point ( cornersL2[i].x, cornersL2[i].y ), 5,  Scalar ( 0 ), 2, 8, 0 );
            }
            // *********************** to display the feature points in the optical flow in the image **********************
            for ( int i=0; i < features_found.size(); i++ )
            {
                Point p0 ( ceil ( cornersL2[i].x ), ceil ( cornersL2[i].y ) );
                Point p1 ( ceil ( cornersL1[i].x ), ceil ( cornersL1[i].y ) );
                line ( tempimageL1 , p0, p1, CV_RGB ( 0,255,255 ), 2 );
            }
            IplImage tmp = tempimageL1;
            outLImg.resize ( tmp.width, tmp.height );
            cvCopyImage ( &tmp, ( IplImage * ) outLImg.getIplImage() );
            imageLOut.write();
            Bottle *human_cmd1 = port_rpc_human.read (false);
	    int first = 0;
            if ( human_cmd1 != NULL ){
	      first = 1;
                strhuman_cmd = human_cmd1->get ( 0 ).asString();
		}
	    if (strhuman_cmd == "Pause"){
	      NULL;
	    }
	    if (strhuman_cmd == "PredictMLball"){
	      check_predict_type = 1;
	      strhuman_cmd = "PredictML";
	      threshold = -1;
	      total_frames_eval = 100;
	    }
            if ( strhuman_cmd == "CollectData" )
            {
	      mu = 0;
	      mv = 0;
	      siguu = 0;
	      sigvv = 0;
	      siguv = 0;
                if ( lock <= trainingdatasize )
                {
                    mu = t.findMu ( 1,cornersL1,cornersL2,features_found );
                    mv = t.findMu ( 2,cornersL1,cornersL2,features_found );
                    siguu = t.findSig ( 1,mu,mv,cornersL1,cornersL2,features_found );
                    sigvv = t.findSig ( 2,mu,mv,cornersL1,cornersL2,features_found );
                    siguv = t.findSig ( 3,mu,mv,cornersL1,cornersL2,features_found );
		    if(velocityArray != 0){
                    writeDataToFile ( velocityArray , mu, mv, siguu, sigvv, siguv );
                    lock ++;
		    }
                }
            }
            else if ( first && strhuman_cmd == "TrainML" )
	    {
	      ml.trainData();
            }
            else if ( strhuman_cmd == "PredictML" )
            {
	      gMat2D<T> Statistics = ml.predictML(velocityArray);
	      double predict_mu = Statistics[0][0];
	      double predict_mv = Statistics[0][1];
	      double predict_siguu = Statistics[0][2];
	      double predict_sigvv = Statistics[0][3];
	      double predict_siguv = Statistics[0][4];
	      int count = features_found.size();
	if (total_frames_eval == 100){
	  if (check_predict_type == 1){
	    file.open("negative_example.csv", std::fstream::app);
	    file<<threshold<<" "<< total_anom_count <<" "<<error_counter<<endl;
	    error_counter = 0;
	  }
	  else{
	 file.open("positive_example.csv", std::fstream::app);
	 file<<threshold<<" "<< total_anom_count <<endl;
	  }
	file.close();
	port.read(head_move_check);
	hm_check = head_move_check.get(0).asInt();
	}
	if (hm_check){
	  threshold ++;
	  hm_check = 0;
	  total_anom_count = 0;
	  total_frames_eval = 0;
	}
	anom_counter = 0;
        for ( int i = 0 ; i < count ; i++ )
        {
            double x = cornersL2[i].x; 
            double y = cornersL2[i].y; 
            double u = cornersL2[i].x - cornersL1[i].x; 
            double v = cornersL2[i].y - cornersL1[i].y; 
            double diffu = u - predict_mu;
            double diffv = v - predict_mv;
            double distance = sqrt(abs(diffu * ( predict_siguu * diffu + predict_siguv * diffv ) + diffv * ( predict_siguv * diffu + predict_sigvv * diffv )));
            if ( distance > threshold )
            {
	     if (check_predict_type == 1){
		    if ((p.x - 30 < cornersL2[i].x) && (p.y + 30 > cornersL2[i].y) && (p.x + 30 > cornersL2[i].x) && (p.y - 30 < cornersL2[i].y)){
		    anom_counter ++;
		    }
		    else{
		    error_counter++;
		    }
		}
		else{
		  anom_counter ++;
		}
                circle ( tempimageL2, Point ( cornersL2[i].x, cornersL2[i].y ), 5,  Scalar ( 150 ), 2, 8, 0 ); //change image
            }
        }
        total_frames_eval++;
        total_anom_count = total_anom_count + anom_counter;
            }
        if (check_predict_type == 1){
	rectangle(tempimageL2,cv::Point (p.x + 30, p.y + 30), cv::Point (p.x - 30, p.y - 30), Scalar ( 0 ), 2, 8, 0 );}
        IplImage tmp12 = tempimageL2;
        outRImg.resize ( tmp12.width, tmp12.height );
        cvCopyImage ( &tmp12, ( IplImage * ) outRImg.getIplImage() );
        imageROut.write();
        }
        return true;
    }

    std::vector<cv::Point2f> harrisCorner2 ( cv::Mat src )
    {
        cv::Mat src2;
        cv::cvtColor ( src, src2, CV_BGR2GRAY );
        int win_size = 15;
        int maxCorners = 80;
        double qualityLevel = 0.05;
        double minDistance = 2.0;
        int blockSize = 3;
        double k = 0.04;
        std::vector<cv::Point2f> cornersA;
        goodFeaturesToTrack ( src2,cornersA,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,true );
        return cornersA;
    }

    std::vector<cv::Point2f> harrisCorner ( cv::Mat src )
    {
        int thresh = 200;
        int max_thresh = 255;
        cv::cvtColor ( src, src_gray, CV_BGR2GRAY );
        cv::Mat dst, dst_norm, dst_norm_scaled, src2;
        src2 = cv::Mat::zeros ( src_gray.size(), CV_32FC1 );
        src_gray.convertTo ( src2, CV_32FC1, 1, 0 );
        dst = cv::Mat::zeros ( src2.size(), CV_32FC1 );
        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;
	
	std::vector<cv::Point2f> cornersA;
        cornerHarris ( src2, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
        // Normalizing
        cv::normalize ( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
        cv::convertScaleAbs ( dst_norm, dst_norm_scaled );
        for ( int j = 0; j < dst_norm.rows ; j++ )
        {
            for ( int i = 0; i < dst_norm.cols; i++ )
            {
                if ( ( int ) dst_norm.at<float> ( j,i ) > thresh )
                {
		  cornersA.push_back(Point2f(i.f,j.f));
//                     circle ( src_gray, Point ( i, j ), 5,  Scalar ( 0 ), 2, 8, 0 );
                }
            }
        }
        return cornersA;
    }

    bool respond ( const Bottle& command, Bottle& reply )
    {
        cout<<"Got something, echo is on"<<endl;
        if ( command.get ( 0 ).asString() =="quit" )
            return false;
        else
            reply=command;
        return true;
    }

   virtual bool configure(ResourceFinder &rf)
    {
        string moduleName = "tracker";
        string inputLNameImage = "/" + moduleName + "/imageL:i";
        imageLIn.open ( inputLNameImage.c_str() );
        string inputRNameImage = "/" + moduleName + "/imageR:i";
        imageRIn.open ( inputRNameImage.c_str() );
        string outputLNameImage = "/" + moduleName + "/imageL:o";
        imageLOut.open ( outputLNameImage.c_str() );
        string outputRNameImage = "/" + moduleName + "/imageR:o";
        imageROut.open ( outputRNameImage.c_str() );
        port.open ( "/moveHead/check:i" );
        port_rpc_human.open ( "/tracker/human:io" );
        lock = 0;
        trainingdatasize = 200;
	file.open("velocityArray.csv", std::fstream::trunc);
	file.close();
	
	file.open("positive_example.csv", std::fstream::trunc);
	file.close();
	file.open("negative_example.csv", std::fstream::trunc);
	file.close();
	file.open("flowstatistics_train.csv", std::fstream::trunc);
	file.close();
    optGaze1.put("device", "remote_controlboard");
    optGaze1.put("local", "/tracker/motor/client");
    optGaze1.put("remote", "/icubSim/head");
    robotHead1.open(optGaze1);
    if (!robotHead1.isValid()) 
    {
        printf("Cannot connect to robot head\n");
        return 1;
    }
    robotHead1.view(pos);
    robotHead1.view(vel);
    robotHead1.view(enc);
    robotHead1.view(encTimed);
    if (pos==NULL || vel==NULL || enc==NULL ) 
    {
        printf("Cannot get interface to robot head\n");
        robotHead1.close();
        return 1;
    }
    jnts = 0;
    pos->getAxes(&jnts);
    Flag = false;
    threshold = -1;
    total_frames_eval = 100;
    return true;
  }

    void writeDataToFile ( double velocityArray ,double mu, double mv, double siguu, double sigvv, double siguv )
    {
	file.open("velocityArray.csv", std::fstream::app);
        file<<velocityArray<<endl;
	file.close();
      	file.open("flowstatistics_train.csv", std::fstream::app);
        file<<mu<<" "<<mv<<" "<<siguu<<" "<<sigvv<<" "<<siguv<<endl;
	file.close();
    }

    bool interruptModule()
    {
        imageLIn.interrupt();
        imageRIn.interrupt();
        imageLOut.interrupt();
        imageROut.interrupt();
	port.interrupt();
        port_rpc_human.interrupt();
        return true;
    }

    bool close()
    {
        port_rpc_human.close();
        imageLIn.close();
        imageLOut.close();
        imageRIn.close();
        imageROut.close();
	port.close();
        return true;
    }
};

int main ( int argc, char *argv[] )
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;
    MyModule mod;
    ResourceFinder rf;
    rf.configure ( argc, argv );
    rf.setVerbose ( true );
    return mod.runModule(rf);
    return 0;
}

