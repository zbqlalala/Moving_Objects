/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
//#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

/* for delaunay*/
#include<opencv2/imgproc/imgproc_c.h>
#include<opencv2/legacy/legacy.hpp>
#include<opencv2/opencv.hpp>  
#include<stdio.h>
#include<typeinfo>


namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

vector<vector<int> > FrameDrawer::Delaunay(vector<CvPoint2D32f> temp_points,vector<float> temp_pIdMapx,vector<float> temp_pIdMapy)
{    
	//Initialization
	
        CvMemStorage* storage;
	CvSubdiv2D* subdiv;
	CvRect rect  = { 0, 0, 640, 480 };// x y width height
	storage = cvCreateMemStorage(0);
	subdiv  = cvCreateSubdivDelaunay2D( rect,storage);
	int count=0;
	
        vector<CvPoint2D32f> points=temp_points;
	vector<float> pIdMapx=temp_pIdMapx,pIdMapy=temp_pIdMapy;
	vector<float>::iterator itpID1=pIdMapx.begin(),itpID2=pIdMapx.begin();
	vector<float>::iterator itpID3=pIdMapy.begin(),itpID4=pIdMapy.begin();
	 //second we delaunay them
	for(int i=0;i<points.size();i++)
        { 
	   CvPoint2D32f fp=points[i];
	   cvSubdivDelaunay2DInsert(subdiv, fp);
	   //cout<<"times::"<<i<<endl; //just to count the times of delaunay
	}
	
	 //last we record the data and line it		 
	 vector< vector<int> > pIdNeighbor;
	 {
              CvSeqReader reader;
	      int total = subdiv->edges->total;//the number of edges    
	      int elem_size = subdiv->edges->elem_size;//the size of the edges

	      cvStartReadSeq((CvSeq*)(subdiv->edges), &reader, 0);
	      //使用CvSeqReader遍历Delaunay  

	      for (int i = 0; i< total; i++)
	      {
		  CvQuadEdge2D* edge= (CvQuadEdge2D*)(reader.ptr);
		      
		      if(CV_IS_SET_ELEM(edge))
		      {
			    CvSubdiv2DPoint* org_pt = cvSubdiv2DEdgeOrg((CvSubdiv2DEdge)edge);
			    CvSubdiv2DPoint* dst_pt = cvSubdiv2DEdgeDst((CvSubdiv2DEdge)edge);
			   
			    //如果两个端点不为空  
			    if (org_pt && dst_pt )
			    {
			        //transformation
			        CvPoint2D32f org = org_pt->pt;
			        CvPoint2D32f dst = dst_pt->pt;
				// cout<<"org.x  "<<org.x<<"  dst.x  "<<dst.x<<endl;
				 
				int IDtemp1=-1,IDtemp2=-1;
			        for (int i=0;  itpID1!= pIdMapx.end(); i++, itpID1++)
			        {
				   if(pIdMapx[i]==org.x&& pIdMapy[i]==org.y)
				   {
				     IDtemp1=i;
				     break;
			           } 
			        }
			        itpID1=pIdMapx.begin();
			        for (int i = 0;  itpID1!= pIdMapx.end(); i++, itpID1++)
			        {
				   if(pIdMapx[i]==dst.x&& pIdMapy[i]==dst.y)
				   {
				     IDtemp2=i;
				     break;
			           } 
			        }
                       
				
				  if(IDtemp1!=-1 && IDtemp2!=-1)
				  {
				      vector<int> vtemp;
				      vtemp.push_back(IDtemp1);
				      vtemp.push_back(IDtemp2);
				      pIdNeighbor.push_back(vtemp);
				      count++;
				  }
				  
				 
				
			    }		      
		      }
		      //we finish this edge and we go to the next edge
		      CV_NEXT_SEQ_ELEM(elem_size, reader);
	      }
	      //finish the cycle for,and we tranvearse all edges.
	}//finish the record
	
	return pIdNeighbor;
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap,vbMoving,vbOutlier; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
	    vbMoving=mvbMoving;
	    vbOutlier=mvbOutlier;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
	mnMoving=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
		
	//in this place   make delaunay~~~~~
	vector<CvPoint2D32f> points;
	vector<float> pIDMapx=vector<float>(n,-1);
	vector<float> pIDMapy=vector<float>(n,-1);
	//map<float,int> pIdMapx,pIdMapy;
	for(int i=0;i<n;i++)
        { //first we have to push the points back; and it si for the vbMap points
	    if(vbMap[i])
            {      
		CvPoint2D32f fp=vCurrentKeys[i].pt;
		points.push_back(fp);
                //pIdMapx.insert(make_pair(fp.x,i));// so this i contains the index of this point in source frame
		//pIdMapy.insert(make_pair(fp.y,i));// to make sure it is the accurate point.
		pIDMapx[i]=fp.x;
		pIDMapy[i]=fp.y;
	    }
	}	  
	  //second we delaunay them	  	  
	  //last we record the data and line it	
	  //and these two steps are done in function delaunay.
	  	
	vector<vector<int> > mvlines = Delaunay(points,pIDMapx,pIDMapy);
	for(int i=0;i<mvlines.size();i++)
	{
	  vector<int> temp=mvlines[i];
	  cv::line(im,vCurrentKeys[temp[0]].pt,vCurrentKeys[temp[1]].pt,cv::Scalar(255,0,0),2);	  
	}
	//finish the delaunay
	
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
	      //说明这个点不是地图点　不是外点
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {		 
		  //original
                   /* cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
		    */
		    
		   //for test1
		    /*if(i>0)
		    {
		       cv::line(im,vCurrentKeys[i-1].pt,vCurrentKeys[i].pt,cv::Scalar(0,255,0));
		    }*/
		    
		    //for test2
		    //because the sysytem goes down,so we comment it for a try		    
		    if(vbMoving[i])
		    {
		      //it is the move point so change the color to red  BGR
		       cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
		       mnMoving++;
		    }else{
		      //original
		        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
		        mnTracked++;
		    }
                   
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame 
                {
		  //对这个我就不进行操作了　因为是VO的　暂时不管？
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            /*else{//just for curious, to see the ouliers
	        cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
		cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);		
	    }*/
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked<<", MovingPs: " << mnMoving;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvbMoving= vector<bool>(N,false);
    mvbOutlier= vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
		    {
                        mvbMap[i]=true;
		    }
                    else
		    {
                        mvbVO[i]=true;
		        cout<<"in FrameDrawer:it is the VO mode"<<endl;
		    }
		    //这个只在VO情况下出现　所以一般也是没有的
                }
                else
		{//just for curious,want to see the distribution of the outliers,and the outliers here depends on the system
		     mvbOutlier[i]=true;
		}
		
                //for test the moving point
                if(pTracker->mCurrentFrame.mvbOutlierForTest[i])
                {
                    mvbMoving[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
