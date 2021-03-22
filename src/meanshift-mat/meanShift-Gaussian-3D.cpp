#include <cstring>
#include <cmath>
#include <cstdlib>
#include "meanShift-Gaussian-3D.hpp"

//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))


#define _2D_to_linear(row, col, maxCol) ((row)*(maxCol)+(col))

const int MAX_DISTANCE = 500;
cv::Mat weight0;

void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterT, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth, cv::Mat TclusterVotes)
{
	std::cout<<"calling gaussian"<<std::endl;
	//Initialization
	int numPts = dataPts.cols;
	int numDim = dataPts.rows;
	int numClust = 0;
	double bandSq = bandwidth*bandwidth;
	cv::Range rng = cv::Range(0,numPts-1);

	cv::Mat onesAux = cv::Mat::ones(1, dataPts.cols, dataPts.type());
	std::cout<<"dataPts.cols="<<dataPts.cols<<std::endl;
	std::vector<int> initPtInds;
	for (int i = rng.start; i <= rng.end; i++) initPtInds.push_back(i);
	double stopThresh = 1E-2*bandwidth; //1E-3*bandwidth; 						//when mean has converged
	int numInitPts = numPts; 								//track if a points been seen already
	cv::Mat beenVisitedFlag = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//number of points to posibaly use as initilization points
	cv::Mat clusterVotes; 		//used to resolve conflicts on cluster membership

	double lambda = 10.;
	double myMeanX, myMeanY, myMeanZ, myMeanT;
	double myOldMeanX, myOldMeanY, myOldMeanZ, myOldMeanT;
	double totalWeightX, totalWeightY, totalWeightZ, totalWeightT;
	double temp1, temp2, temp3, temp4, myMeanX1, myMeanY1, myMeanT1, myMeanZ1;
	double temp11, temp22, temp33, temp44;
	double dev_myMeanX, dev_myMeanY, dev_myMeanZ, dev_myMeanT, dev_totalWeightX, dev_totalWeightY, dev_totalWeightZ, dev_totalWeightT;	

	int stInd;

	int tempInd;

	std::cout<<"start while"<<std::endl;
	// while (numInitPts>0)
	// {

		tempInd = (rand()%numInitPts);	 //pick a random seed point
		//ROS_ERROR_STREAM("numInitPts ="<<numInitPts);

		stInd = initPtInds[tempInd];	//use this point as start of mean
		myMeanT = dataPts.at<double>(cv::Point(stInd, 0));
		myMeanX = dataPts.at<double>(cv::Point(stInd, 1));//intilize mean to this points location
		myMeanY = dataPts.at<double>(cv::Point(stInd, 2));
		myMeanZ = dataPts.at<double>(cv::Point(stInd, 3));
		cv::Mat thisClusterVotes = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); //used to resolve conflicts on cluster membership
		//ROS_ERROR_STREAM("Before getting into while myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
		cv::Mat_<double> myMean = ( cv::Mat_<double>(4, 1) << myMeanT, myMeanX, myMeanY, myMeanZ);

		std::cout<<"start while (true)"<<std::endl;

		int while_iter=0;
		bool while_sign=true;
		while(while_sign)
		{
			while_iter++;
			cv::Mat myDataPts; //3xcols;
			dataPts.copyTo(myDataPts);
			std::cout<<"111"<<std::endl;
			std::cout<<"myDataPts.rows="<<myDataPts.rows<<",cols="<<myDataPts.cols<<std::endl;
			std::cout<<"onesAux.rows="<<onesAux.rows<<",cols="<<onesAux.cols<<std::endl;
			std::cout<<"myMean.rows="<<myMean.rows<<",cols="<<myMean.cols<<std::endl;
			cv::Mat diff = myDataPts - myMean*onesAux; //3xcols; myMean:3x1; //myMean*onesAux: after shift;
			std::cout<<"111-1"<<std::endl;
			cv::Mat diffdiff = diff.mul(diff); //3xcols;
			std::cout<<"111-2"<<std::endl;
			cv::Mat sqDistToAll = diffdiff.row(0) +diffdiff.row(1) + diffdiff.row(2) + diffdiff.row(3);//1xcols;	//dist squared from mean to all points still active
			//x1+y1+z1-x2-y2-z2; 1*cols (186)
			std::cout<<"222"<<std::endl;
			std::cout<<"sqDistToAll="<<sqDistToAll<<std::endl;
			std::cout<<"bandsq="<<bandwidth*bandwidth*bandwidth<<","<<bandwidth*bandwidth<<","<<bandwidth<<std::endl;
			// double dev_diff=(double)(cv::sum(diff.row(0)+diff.row(1)+diff.row(2)+diff.row(3))[0]);//sum(x-x')
			std::cout<<"333"<<std::endl;
			cv::Mat inInds = (sqDistToAll < bandSq); // Puts 255 wherever it is true
			cv::add(thisClusterVotes, cv::Scalar(1), thisClusterVotes, inInds); //add a vote for all the in points belonging to this cluster //thisClusterVotes: 1xcols; // if (inInds==1), thisClusterVotes= 1 + thisClusterVotes;
			cv::Mat mask4 = cv::repeat(inInds==0, 4, 1); // CV_EXPORTS Mat repeat(const Mat& src, int ny, int nx);--> 3xcols;
			std::cout<<"444"<<std::endl;
			myDataPts.setTo(0, mask4); 
			double numOfOnes = (double) (cv::sum(inInds)[0])/255; //inInds is active if inInds[i]==255

			myOldMeanT = myMeanT; myOldMeanX = myMeanX; myOldMeanY = myMeanY; myOldMeanZ = myMeanZ;

			myMeanT = cv::sum(myDataPts.row(0))[0]/numOfOnes;
			myMeanX = cv::sum(myDataPts.row(1))[0]/numOfOnes;
			myMeanY = cv::sum(myDataPts.row(2))[0]/numOfOnes;
			myMeanZ = cv::sum(myDataPts.row(3))[0]/numOfOnes;

			std::cout<<"get previous mean"<<std::endl;
			myMean = ( cv::Mat_<double>(4, 1) << myMeanT, myMeanX, myMeanY, myMeanZ);
			std::cout<<"555"<<std::endl;
			//ROS_ERROR_STREAM("MIDDLE myMean = "<<myMean);

			diff = myDataPts - myMean*onesAux;
			diffdiff = diff.mul(diff);
			cv::Mat weight; std::cout<<"666"<<std::endl;//std::cout<<"diffdiff="<<diffdiff/lambda<<std::endl;
			cv::Mat tr=diffdiff/lambda;
			cv::exp(tr, weight);
			//weight=0.9*weight+0.1*weight0;   //-------------------------learning rate for updating weight
			std::cout<<"777"<<std::endl;
			// gradient;
			// cv::Mat dev_weight;
			// dev_weight=weight.mul(dev_diff)*(2);//(dev_diff);
			//std::cout<<"weights="<<(double) (cv::sum(weight)[0])<<" dweights="<<(double) (cv::sum(dev_weight)[0])<<std::endl;

			weight.setTo(0., mask4); //Again, because the exp and the x - u make invalid values non-zero (inInds)
			// dev_weight.setTo(0., mask4);
			cv::Mat wData = weight.mul(myDataPts);
			std::cout<<"888"<<std::endl;
			// cv::Mat dev_wData = dev_weight.mul(myDataPts);

			totalWeightT = cv::sum(weight.row(0))[0];
			totalWeightX = cv::sum(weight.row(1))[0];
			totalWeightY = cv::sum(weight.row(2))[0];
			totalWeightZ = cv::sum(weight.row(3))[0];
			myMeanT = cv::sum(wData.row(0))[0]/totalWeightT;
			myMeanX = cv::sum(wData.row(1))[0]/totalWeightX;
			myMeanY = cv::sum(wData.row(2))[0]/totalWeightY;
			myMeanZ = cv::sum(wData.row(3))[0]/totalWeightZ;
			std::cout<<"999"<<std::endl;
			// dev_totalWeightT = cv::sum(dev_weight.row(0))[0];
			// dev_totalWeightX = cv::sum(dev_weight.row(1))[0];
			// dev_totalWeightY = cv::sum(dev_weight.row(2))[0];
			// dev_totalWeightZ = cv::sum(dev_weight.row(3))[0];
			//dev_myMeanX = (dev_totalWeightX==0) ?0:sum(dev_wData.row(0))[0]/dev_totalWeightX;//-myOldMeanX;
			//dev_myMeanY = (dev_totalWeightY==0) ?0:sum(dev_wData.row(1))[0]/dev_totalWeightY;//-myOldMeanY;
			//dev_myMeanZ = (dev_totalWeightZ==0) ?0:sum(dev_wData.row(2))[0]/dev_totalWeightZ;//-myOldMeanZ;
			// dev_myMeanT = cv::sum(dev_wData.row(0))[0]/dev_totalWeightT;//-myOldMeanX;
			// dev_myMeanX = cv::sum(dev_wData.row(1))[0]/dev_totalWeightX;//-myOldMeanX;
			// dev_myMeanY = cv::sum(dev_wData.row(2))[0]/dev_totalWeightY;//-myOldMeanY;
			// dev_myMeanZ = cv::sum(dev_wData.row(3))[0]/dev_totalWeightZ;//-myOldMeanZ;

			// temp1 = myMeanT+0.5*(myMeanT-myOldMeanT);
			// temp2 = myMeanX+0.5*(myMeanX-myOldMeanX);//dev_myMeanX; //0.5*(myMeanX-myOldMeanX);
			// temp3 = myMeanY+0.5*(myMeanY-myOldMeanY);//dev_myMeanY; //0.5*(myMeanY-myOldMeanY);
			// temp4 = myMeanZ+0.5*(myMeanZ-myOldMeanZ);//dev_myMeanZ; //0.5*(myMeanZ-myOldMeanZ);

			// temp11 = dev_myMeanT+0.1*(dev_myMeanT-myOldMeanT);
			// temp22 = dev_myMeanX+0.1*(dev_myMeanX-myOldMeanX);//dev_myMeanX; //0.5*(myMeanX-myOldMeanX);
			// temp33 = dev_myMeanY+0.1*(dev_myMeanY-myOldMeanY);//dev_myMeanY; //0.5*(myMeanY-myOldMeanY);
			// temp44 = dev_myMeanZ+0.1*(dev_myMeanZ-myOldMeanZ);//dev_myMeanZ; //0.5*(myMeanZ-myOldMeanZ);

			// myMeanT=temp11;
			// myMeanX=temp22;
			// myMeanY=temp33;
			// myMeanZ=temp44;

			/*temp1 = min(temp1,1);
			temp2 = min(temp2,1);
			temp3 = min(temp3,1);*/

			//std::cout<<"mean="<<myMeanX<<",temp="<<temp1<<std::endl;

			// myMeanT1 = myMeanT+0.05*dev_myMeanT;
			// myMeanX1 = myMeanX+0.05*dev_myMeanX; 
			// myMeanY1 = myMeanY+0.05*dev_myMeanY;
			// myMeanZ1 = myMeanZ+0.05*dev_myMeanZ;

			//std::cout<<"mean="<<myMeanX<<",temp="<<temp1<<",dev="<<myMeanX1<<std::endl;//dev_myMeanX

			//std::cout<<"wData="<<sum(wData.row(0))[0]<<",dev_wData="<<sum(dev_wData.row(0))[0]<<std::endl;//-------------
			//std::cout<<"totalWeightX="<<totalWeightX<<",dev_totalWeightX="<<dev_totalWeightX<<std::endl;
			//std::cout<<"x="<<myMeanX<<"y="<<myMeanY<<"z="<<myMeanZ<<std::endl;
			//std::cout<<"dev1="<<dev_myMeanX<<"dev2="<<dev_myMeanY<<"dev3="<<dev_myMeanZ<<std::endl;

			myMean = ( cv::Mat_<double>(4, 1) << myMeanT, myMeanX, myMeanY, myMeanZ);
			//myMean = ( cv::Mat_<double>(3, 1) << dev_myMeanX, dev_myMeanY, dev_myMeanZ);
			//myMean = ( cv::Mat_<double>(3, 1) << temp1, temp2, temp3);
			//myMean = ( cv::Mat_<double>(4, 1) << temp11, temp22, temp33, temp44);
			//myMean = ( cv::Mat_<double>(3, 1) << myMeanX1, myMeanY1, myMeanZ1);
			//ROS_ERROR_STREAM("AFTER: myMean = "<<myMeanX<<","<<myMeanY<<","<<myMeanZ);
			//exit(0);
			
			beenVisitedFlag.setTo(1, inInds);

			std::cout<<"long="<<(myMeanT-myOldMeanT)*(myMeanT-myOldMeanT) + \
				(myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) \
				+ (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ)<<std::endl;

			std::cout<<"stopThresh*stopThresh="<<stopThresh*stopThresh<<std::endl;
			
			// if mean doesn't move much stop this cluster
			if((myMeanT-myOldMeanT)*(myMeanT-myOldMeanT) + (myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) + (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ) < stopThresh*stopThresh*10)
			{
				//check for merge posibilities
				//ROS_ERROR_STREAM("Dentro!! ");
				int mergeWith = -1;
				double distToOther;
				for(int cN = 0; cN<numClust; cN++) //Careful!! cN goes from 1 to numClust!!!
				{
					double distToOther = (myMeanT - (*clusterCenterT)[cN])*(myMeanT - (*clusterCenterT)[cN])+(myMeanX - (*clusterCenterX)[cN])*(myMeanX - (*clusterCenterX)[cN]) + (myMeanY - (*clusterCenterY)[cN])*(myMeanY - (*clusterCenterY)[cN]) + (myMeanZ - (*clusterCenterZ)[cN])*(myMeanZ - (*clusterCenterZ)[cN]); //distance from posible new clust max to old clust max
					std::cout<<"distToOther="<<distToOther<<std::endl;
					if(distToOther < (bandwidth/2))//*(bandwidth/2))  //if its within bandwidth/2 merge new and old
					{
						mergeWith = cN;
						break;
					}
				}
				if(mergeWith > -1)
				{
					(*clusterCenterT)[mergeWith] = 0.5*(myMeanT+(*clusterCenterT)[mergeWith]);
					(*clusterCenterX)[mergeWith] = 0.5*(myMeanX+(*clusterCenterX)[mergeWith]);
					(*clusterCenterY)[mergeWith] = 0.5*(myMeanY+(*clusterCenterY)[mergeWith]);
					(*clusterCenterZ)[mergeWith] = 0.5*(myMeanZ+(*clusterCenterZ)[mergeWith]);

					cv::Mat newClusterVotes = cv::Mat(clusterVotes.row(mergeWith)) + thisClusterVotes;
					newClusterVotes.copyTo(clusterVotes.row(mergeWith));
				}
				else
				{
					(*clusterCenterT).push_back(myMeanT);    //record the mean value
					(*clusterCenterX).push_back(myMeanX);
					(*clusterCenterY).push_back(myMeanY);
					(*clusterCenterZ).push_back(myMeanZ);
					numClust = numClust+1;                   //increment clusters

					clusterVotes.push_back(thisClusterVotes);  // add the new row for the new cluster

				}
				while_sign=false;
				// break;
			} // end-if moving distance < threshold
			//weight0=weight.clone();
			std::cout<<"break_iter_"<<while_iter<<",while_sign="<<while_sign<<std::endl;
		} // while (true)------------
		std::cout<<"end while (true)"<<std::endl;
		//ROS_ERROR_STREAM("Number of clusterCenterX after while= "<<(*clusterCenterX).size());
		std::vector<int> newInitPtInds;	//points not yet visited
		ROS_INFO("numPts=%d",numPts);
		for(int i=0; i<numPts; i++)
			if (beenVisitedFlag.at<int>(cv::Point(i, 0)) == 0)
				newInitPtInds.push_back(i);
		initPtInds = newInitPtInds;
		numInitPts = initPtInds.size(); //number of active points in set
		
	// }// while (numInitPts>0)
	ROS_INFO("After While");
	TclusterVotes = clusterVotes.t(); //TclusterVotes=[cluster_num x points_num in each cluster]
	cv::Point minLoc;
	cv::Point maxLoc;
	double min, max;
		std::cout<<"TclusterVotes(i).size="<<TclusterVotes.rows<<std::endl;
	for(int i=0; i<TclusterVotes.rows; i++)
	{
		cv::minMaxLoc(cv::Mat(TclusterVotes.row(i)), &min, &max, &minLoc, &maxLoc);
		(*point2Clusters).push_back(maxLoc.x);
		/*std::cout<<"TclusterVotes"<<i<<"=";
		for (int j=0; j<TclusterVotes.cols; j++)	
		{
			//if (TclusterVotes.at<double>(i,j)<0.001) TclusterVotes.at<double>(i,j)=0;
			std::cout<<TclusterVotes.at<double>(i,j)<<",";
		}
		std::cout<<"."<<std::endl;*/
	}
	ROS_INFO("Number of clusters =%d, %d", numClust, (*clusterCenterX).size());
	// ROS_ERROR_STREAM("Tclusters rows " <<TclusterVotes.rows<<" cols "<<TclusterVotes.cols<<",TclusterVotes.size="<<TclusterVotes.size()<<",clusterVotes.size="<<clusterVotes.size()<<",point2Clusters.size="<<point2Clusters->size()<<","<<TclusterVotes.at<double>(1,6));
	

}

