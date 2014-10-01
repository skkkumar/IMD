
#include <yarp/os/RFModule.h>
#include "gurls++/gurls.h"
#include <yarp/dev/all.h>
#include <cv.h>
#include <ml.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace cv;
using namespace gurls;

// #define harriscorner2_win_size 5
#define harriscorner2_maxCorners 100
#define harriscorner2_qualityLevel 0.02
#define harriscorner2_minDistance 3.0
#define harriscorner2_blockSize 3
#define harriscorner2_k 0.04
#define Box_size_value 18
#define trainingdatasize_value 500
#define total_frames_eval_value 100
#define total_no_of_velocityArray 6


typedef double T;

class ML{
      GURLS G;
      GurlsOptionsList* opt;
      OptTaskSequence *seq;
      GurlsOptionsList * process;
      OptString* hofun;
      OptFunction* optfun;
public:
  void trainData(){
    gMat2D<T> ML_velocityArray, ML_flowstatistics_train;
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

  gMat2D<T> predictML(gMat2D<T> ML_velocityArray_test){
//     gMat2D<T> ML_velocityArray_test(1,sizeof(velocityArray)/8);
    gMat2D<T> ML_flowstatistics_test(1,5);
//     ML_velocityArray_test=velocityArray;
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
    BufferedPort<Bottle> roc_value_out;
    Port port;
    String strhuman_cmd;
    String strhuman_cmd_ext;
    cv::Mat src_gray;
    double bottlevector1[total_no_of_velocityArray];
    double bottlevector2[total_no_of_velocityArray];
    double velocityArray[total_no_of_velocityArray];
    cv::Mat imageL1;
    cv::Mat imageL2;
    cv::Mat imageR1;
    cv::Mat imageR2;
    int lock;
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
    int anom_counter,total_inside_box;
    int error_counter,total_outside_box;
    int check_predict_type;
    int first;
    double px,py;
    int maxCorners;
        double qualityLevel;
        double minDistance;
        int blockSize ;
        double k ;
    int Box_size, trainingdatasize;
    int velocityArray_pos[total_no_of_velocityArray];
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

    double returnVelocity(int pos)
    {
      enc->getEncoder(pos, &position);
      return position;
    }

    void imageAcq()
    {
        if ( imageL1.data == NULL and imageR1.data == NULL )
        {
            imageL1 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR1 = ( IplImage * ) imageRIn.read()->getIplImage();
	    for (int n = 0; n < total_no_of_velocityArray; n ++){
            bottlevector1[n] = returnVelocity(velocityArray_pos[n]);}
            imageL2 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR2 = ( IplImage * ) imageRIn.read()->getIplImage();
	    for (int n = 0; n < total_no_of_velocityArray; n ++){
            bottlevector2[n] = returnVelocity(velocityArray_pos[n]);}
        }
        else
        {
            imageL1 = imageL2;
            imageR1 = imageR2;
	    for (int n = 0; n < total_no_of_velocityArray; n ++){
            bottlevector1[n] = bottlevector2[n];}
            imageL2 = ( IplImage * ) imageLIn.read()->getIplImage();
            imageR2 = ( IplImage * ) imageRIn.read()->getIplImage();
	    for (int n = 0; n < total_no_of_velocityArray; n ++){
            bottlevector2[n] = returnVelocity(velocityArray_pos[n]);}
        }
    }

    void findDiffVelocity()
    {
      for (int n = 0; n < total_no_of_velocityArray; n ++){
        if ( bottlevector1[n] != 0 and bottlevector2[n] != 0 )
        {
	  velocityArray[n] = bottlevector2[n] - bottlevector1[n];
        }
	
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
	    imageL2.copyTo(tempimageL2);
            findDiffVelocity();
	    cv::Point p = findBall(tempimageL2);
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
            if ( human_cmd1 != NULL ){

                strhuman_cmd = human_cmd1->get ( 0 ).asString();
		}
	    if (strhuman_cmd == "Pause"){
	      NULL;
	    }
	    if (strhuman_cmd == "Set"){
	         if ( human_cmd1 != NULL ){
                strhuman_cmd_ext = human_cmd1->get ( 1 ).asString();
		}
		else {
		  strhuman_cmd_ext = "";
		}
// 	      if (strhuman_cmd_ext == "win_size"){
// 		       win_size = human_cmd1->get ( 2 ).asInt();;

// 		
// 	      }
	      	      if (strhuman_cmd_ext == "maxCorners"){
		        
		maxCorners = human_cmd1->get ( 2 ).asInt();;
	      }
	      	      if (strhuman_cmd_ext == "qualityLevel"){
		
		        qualityLevel = human_cmd1->get ( 2 ).asDouble();;

	      }
	      	      if (strhuman_cmd_ext == "minDistance"){
		
		        minDistance = human_cmd1->get ( 2 ).asDouble();;

	      }
	      	      if (strhuman_cmd_ext == "blockSize"){
		        blockSize = human_cmd1->get ( 2 ).asInt();;
        
		
	      }
	      	      	      if (strhuman_cmd_ext == "k"){
		 k = human_cmd1->get ( 2 ).asDouble();;
		
	      }
	     if (strhuman_cmd_ext == "Box_size"){
		
		Box_size = human_cmd1->get ( 2 ).asInt();
	      }
	     if (strhuman_cmd_ext == "trainingdatasize"){
		
		trainingdatasize = human_cmd1->get ( 2 ).asInt();
	      }
      if (strhuman_cmd_ext == "total_frames_eval"){
		total_frames_eval = human_cmd1->get ( 2 ).asInt();
	      }
	    }
	    if (strhuman_cmd == "PredictMLball"){
	      check_predict_type = 1;
	      strhuman_cmd = "PredictML";
	      threshold = 0;
	    }
	    if (strhuman_cmd == "PredictMLwith")
	    {
	      check_predict_type = 2;
	      strhuman_cmd = "PredictML";
	      threshold = human_cmd1->get ( 1 ).asDouble();
	    }
            if (strhuman_cmd == "PredictMLball2")
	    {
	      check_predict_type = 3;
	      strhuman_cmd = "PredictML";
	      threshold = 0;
	     error_counter = 0;
	    total_inside_box = 0;
	    total_outside_box = 0;
	    total_anom_count = 0;
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
		    if(velocityArray[0] != 0){
                    writeDataToFile ( );
                    lock ++;
		    }
                }
                else
		{
		  if (!first){
		  cout<<"Data Collected for Training";
		  cout <<endl<<flush;
		  first = 1;
		  }
		}
            }
            else if ( first && strhuman_cmd == "TrainML" )
	    {
	      ml.trainData();
	      first = 0;
            }
            else if ( strhuman_cmd == "PredictML" )
            {
	      gMat2D<T> SendData(1,total_no_of_velocityArray);
// 	      SendData.operator+(velocityArray[0]);
	      for (int n = 0; n < total_no_of_velocityArray; n++){
	      SendData(0,n) = velocityArray[n];
	      }
	      gMat2D<T> Statistics = ml.predictML(SendData);
	      double predict_mu = Statistics[0][0];
	      double predict_mv = Statistics[0][1];
	      double predict_siguu = Statistics[0][2];
	      double predict_sigvv = Statistics[0][3];
	      double predict_siguv = Statistics[0][4];
	      int count = features_found.size();
	      
	if (check_predict_type == 2){
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
                circle ( tempimageL2, Point ( cornersL2[i].x, cornersL2[i].y ), 5,   CV_RGB ( 255,0,255 ), 2, 8, 0 ); //change image
            }
        }
	}
	 else if(check_predict_type == 3){
	   
	   while (threshold <= 60){
// 	    for (threshold = 0; threshold < 60;threshold++){
	      	if (total_frames_eval == total_frames_eval_value){
	    file.open("negative_example.csv", std::fstream::app);
	    file<<threshold<<","<<total_inside_box<<","<< total_anom_count <<","<<total_outside_box<<","<<error_counter<<endl;
	    
	    if (total_inside_box == 0)
	    {
	      total_inside_box = 1;
	    }
	    if (total_anom_count == 0)
	    {
	      total_anom_count = 1;
	    }
	    if (total_outside_box == 0)
	    {
	      total_outside_box = 1;
	    }
	    if (error_counter == 0)
	    {
	      error_counter = 1;
	    }
	    cout<<threshold<<","<<total_inside_box<<","<< total_anom_count <<","<<total_outside_box<<","<<error_counter<<endl<<flush;
	    if (total_inside_box>=total_anom_count and total_outside_box >= error_counter){
	      cout << "I am here" << endl<< flush;

	    double data1 = (double)total_anom_count/(double)total_inside_box;
	    double data2 = (double)error_counter/(double)total_outside_box;
	    cout << data1<<" --------------------------------------------- "<<data2<<endl <<flush;
	    Bottle &roc_value = roc_value_out.prepare();
	    
	    roc_value.addDouble(data1);
	    roc_value.addDouble(data2);
	    roc_value.addInt(threshold);
// 	    cout << "before writing"<<endl<<flush;
	    roc_value_out.write();
	    }
	    error_counter = 0;
	    total_inside_box = 0;
	    total_outside_box = 0;
	    total_anom_count = 0;
	file.close();
// 	port.read(head_move_check);
	hm_check = 1;
	}
	if (hm_check){
	  threshold ++;
	  hm_check = 0;
	  total_anom_count = 0;
	  total_frames_eval = 0;
	}
	      
	      
	      
	      
	for ( int i = 0 ; i < count ; i++ )
        {
            double x = cornersL2[i].x; 
            double y = cornersL2[i].y; 
            double u = cornersL2[i].x - cornersL1[i].x; 
            double v = cornersL2[i].y - cornersL1[i].y; 
            double diffu = u - predict_mu;
            double diffv = v - predict_mv;
            double distance = sqrt(abs(diffu * ( predict_siguu * diffu + predict_siguv * diffv ) + diffv * ( predict_siguv * diffu + predict_sigvv * diffv )));
	    file.open("positive_example.csv", std::fstream::app);
	     if ((p.x - Box_size < cornersL2[i].x) && (p.y + Box_size > cornersL2[i].y) && (p.x + Box_size > cornersL2[i].x) && (p.y - Box_size < cornersL2[i].y)){
		    total_inside_box ++;
		    file<<1<<","<<distance<<endl<<flush;
		    }
		    else{
		    total_outside_box++;
		      file<<0<<","<<distance<<endl<<flush;
		    }
		file.close();
            if ( distance > threshold )
            {
	      if ((p.x - Box_size < cornersL2[i].x) && (p.y + Box_size > cornersL2[i].y) && (p.x + Box_size > cornersL2[i].x) && (p.y - Box_size < cornersL2[i].y)){
		    px = px + cornersL2[i].x;
		    py = py + cornersL2[i].y;
		    anom_counter ++;
		    }
		    else{
		    error_counter++;
		    }
//                 circle ( tempimageL2, Point ( cornersL2[i].x, cornersL2[i].y ), 5,   CV_RGB ( 255,0,255 ), 2, 8, 0 ); //change image
            }
	    }
	    total_frames_eval++;
	    total_anom_count = total_anom_count + anom_counter;
	    anom_counter = 0;
	    }
	    if (threshold > 60){
	    strhuman_cmd = "Pause";}
	}
	else {
	if (total_frames_eval == total_frames_eval_value){
	  if (check_predict_type == 1){
	    file.open("negative_example.csv", std::fstream::app);
	    file<<threshold<<","<<total_inside_box<<","<< total_anom_count <<","<<total_outside_box<<","<<error_counter<<endl;
	    
	    if (total_inside_box == 0)
	    {
	      total_inside_box = 1;
	    }
	    if (total_anom_count == 0)
	    {
	      total_anom_count = 1;
	    }
	    if (total_outside_box == 0)
	    {
	      total_outside_box = 1;
	    }
	    if (error_counter == 0)
	    {
	      error_counter = 1;
	    }
// 	    cout<<threshold<<","<<total_inside_box<<","<< total_anom_count <<","<<total_outside_box<<","<<error_counter<<endl<<flush;
	    if (threshold != 0){
	      cout << "I am here" << endl<< flush;

	    double data1 = (double)total_anom_count/(double)total_inside_box;
	    double data2 = (double)error_counter/(double)total_outside_box;
	    cout << data1<<" --------------------------------------------- "<<data2<<endl <<flush;
	    Bottle &roc_value = roc_value_out.prepare();
	    
	    roc_value.addDouble(data1);
	    roc_value.addDouble(data2);
	    roc_value.addInt(threshold);
// 	    cout << "before writing"<<endl<<flush;
	    roc_value_out.write();
	    }
	    
	    error_counter = 0;
	    total_inside_box = 0;
	    total_outside_box = 0;
	  }
	  else{
	 file.open("positive_example.csv", std::fstream::app);
	 file<<threshold<<","<< total_anom_count <<endl;
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
	px = 0;
	py = 0;
        for ( int i = 0 ; i < count ; i++ )
        {
            double x = cornersL2[i].x; 
            double y = cornersL2[i].y; 
            double u = cornersL2[i].x - cornersL1[i].x; 
            double v = cornersL2[i].y - cornersL1[i].y; 
            double diffu = u - predict_mu;
            double diffv = v - predict_mv;
            double distance = sqrt(abs(diffu * ( predict_siguu * diffu + predict_siguv * diffv ) + diffv * ( predict_siguv * diffu + predict_sigvv * diffv )));
	    if (check_predict_type == 1){
		    if ((p.x - Box_size < cornersL2[i].x) && (p.y + Box_size > cornersL2[i].y) && (p.x + Box_size > cornersL2[i].x) && (p.y - Box_size < cornersL2[i].y)){
		    total_inside_box ++;
		    }
		    else{
		    total_outside_box++;
		    }
		}
            if ( distance > threshold )
            {
	     if (check_predict_type == 1){
		    if ((p.x - Box_size < cornersL2[i].x) && (p.y + Box_size > cornersL2[i].y) && (p.x + Box_size > cornersL2[i].x) && (p.y - Box_size < cornersL2[i].y)){
		    px = px + cornersL2[i].x;
		    py = py + cornersL2[i].y;
		    anom_counter ++;
		    }
		    else{
		    error_counter++;
		    }
		}
		else{
		  anom_counter ++;
		}
                circle ( tempimageL2, Point ( cornersL2[i].x, cornersL2[i].y ), 5,   CV_RGB ( 255,0,255 ), 2, 8, 0 ); //change image
            }
            
        }
        total_frames_eval++;
        total_anom_count = total_anom_count + anom_counter;
	cout<<px<<" - "<<py<<endl<<flush;
	circle ( tempimageL2, Point ( px/anom_counter, py/anom_counter ), 5,   CV_RGB ( 0,255,255 ), 2, 8, 0 );
            }
        if (check_predict_type == 1){
	rectangle(tempimageL2,cv::Point (p.x + Box_size, p.y + Box_size), cv::Point (p.x - Box_size, p.y - Box_size), Scalar ( 0 ), 2, 8, 0 );}
	    }
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
        std::vector<cv::Point2f> cornersA;
        goodFeaturesToTrack( src2,cornersA,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,true,k);
        return cornersA;
    }
    
    std::vector<cv::Point2f> harrisCorner ( cv::Mat src )
    {
        int thresh = 220;
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
		  cornersA.push_back(Point2f(i,j));
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
	roc_value_out.open("/roc:o");
        port_rpc_human.open ( "/tracker/human:io" );
        lock = 0;
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
	threshold = 0;
	first = 0;
	total_frames_eval = total_frames_eval_value;
	px = 0;
	py = 0;
	Box_size = Box_size_value;
// 	 win_size = harriscorner2_win_size;
         maxCorners = harriscorner2_maxCorners;
         qualityLevel = harriscorner2_qualityLevel;
         minDistance = harriscorner2_minDistance;
         blockSize = harriscorner2_blockSize;
         k = harriscorner2_k;
	 trainingdatasize = trainingdatasize_value;
	 total_frames_eval = total_frames_eval_value;
// 	 velocityArray = new int[2];
	 velocityArray_pos[0] = 3;
	 velocityArray_pos[1] = 4;
	return true;
  }

    void writeDataToFile ( )
    {
	file.open("velocityArray.csv", std::fstream::app);
	cout << "total_no_of_velocityArray : " << total_no_of_velocityArray << endl << flush; 
	for (int n = 0; n < total_no_of_velocityArray; n ++){
	  cout<<velocityArray[n]<<" ";
        file<<velocityArray[n]<<" ";
	}
	cout<<endl;
	file<<endl;
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

