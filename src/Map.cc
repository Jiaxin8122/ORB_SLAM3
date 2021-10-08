/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Map.h"
#include "log.h"
#include "Converter.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),mnLastLoopKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mnLastLoopKFid = 0;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

void Map::RotateMap(const cv::Mat &R)
{
    unique_lock<mutex> lock(mMutexMap);

    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    KeyFrame* pKFini = mvpKeyFrameOrigins[0];
    cv::Mat Twc_0 = pKFini->GetPoseInverse();
    cv::Mat Txc_0 = Txw*Twc_0;
    cv::Mat Txb_0 = Txc_0*pKFini->mImuCalib.Tcb;
    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);
    Tyx.rowRange(0,3).col(3) = -Txb_0.rowRange(0,3).col(3);
    cv::Mat Tyw = Tyx*Txw;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        pKF->SetVelocity(Ryw*Vw);
    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
}

void Map::ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel, const cv::Mat t)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Tyw = Tyx*Txw;
    Tyw.rowRange(0,3).col(3) = Tyw.rowRange(0,3).col(3)+t;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        Twc.rowRange(0,3).col(3)*=s;
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s*Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::PrintEssentialGraph()
{
    //Print the essential graph
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    cout << "KF: " << pFirstKF->mnId << endl;
    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    vector<KeyFrame*> vpChilds;
    vector<string> vstrHeader;
    for(KeyFrame* pKFi : spChilds){
        vstrHeader.push_back("--");
        vpChilds.push_back(pKFi);
    }
    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        string strHeader = vstrHeader[i];
        KeyFrame* pKFi = vpChilds[i];

        cout << strHeader << "KF: " << pKFi->mnId << endl;

        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        for(KeyFrame* pKFj : spKFiChilds)
        {
            vpChilds.push_back(pKFj);
            vstrHeader.push_back(strHeader+"--");
        }
    }
    if (count == (mspKeyFrames.size()+10))
        cout << "CYCLE!!"    << endl;

    cout << "------------------" << endl << "End of the essential graph" << endl;
}

bool Map::CheckEssentialGraph(){
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    cout << "Checking if the first KF has parent" << endl;
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    vector<KeyFrame*> vpChilds;
    vpChilds.reserve(mspKeyFrames.size());
    for(KeyFrame* pKFi : spChilds)
        vpChilds.push_back(pKFi);

    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        KeyFrame* pKFi = vpChilds[i];
        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        for(KeyFrame* pKFj : spKFiChilds)
            vpChilds.push_back(pKFj);
    }

    cout << "count/tot" << count << "/" << mspKeyFrames.size() << endl;
    if (count != (mspKeyFrames.size()-1))
        return false;
    else
        return true;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this)
            {
                pMPi->EraseObservation(it->first); //We need to find where the KF is set as Bad but the observation is not removed
            }

        }
    }
    cout << "  Bad MapPoints removed" << endl;

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    mvpBackupMapPoints.clear();
    // Backup of set container to vector
    //std::copy(mspMapPoints.begin(), mspMapPoints.end(), std::back_inserter(mvpBackupMapPoints));
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Pre-save of mappoint " << pMPi->mnId << endl;
        //mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);

        //XLOG_DEBUG("[SAVE]wold pose of MP: {}", pMPi->GetWorldPos());
    }
    cout << "  MapPoints back up done!!" << endl;

    // Debug Test
//    int end = 0;
//    for(set<MapPoint *>::iterator it = mspMapPoints.begin(); end != 10; end++, it++)
//    {
//        MapPoint * mMPi = *it;
//        cout << "[LOAD_AFTER]wold pose of MP: " << mMPi->GetWorldPos() << endl;
//
//    }

    std::string filename = "./log/MP_save.txt";

    std::cout << std::endl << "Saving MP to txt ..." << std::endl;

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

//    for(MapPoint* pMPi : mspMapPoints){
//        f << "Map Point Pose: " << pMPi->GetWorldPos() << endl;
//    }

    for(KeyFrame* pKFi : mspKeyFrames){
        f << "KeyFrame Pose: " << pKFi->GetPose() << endl;
    }

    f.close();


    mvpBackupKeyFrames.clear();
    //std::copy(mspKeyFrames.begin(), mspKeyFrames.end(), std::back_inserter(mvpBackupKeyFrames));
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        //mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
        //XLOG_DEBUG("SAVE:: connected key frames numbers: {}", pKFi->GetConnectedKeyFrames().size());
    }
    cout << "  KeyFrames back up done!!" << endl;

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams, const string& strSettingPath)
{

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        cout << "can't open setting file with yaml";
    }


    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));


    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
//        pKFi->ComputeBoW();
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    cout << "Number of KF: " << mspKeyFrames.size() << endl;
    cout << "Number of MP: " << mspMapPoints.size() << endl;

    // References reconstruction between different instances

//    std::string filename_before_KF = "./log/KF_before.txt";
//
//    std::cout << std::endl << "Saving KF_before to txt ..." << std::endl;
//
//    std::ofstream f_before_KF;
//    f_before_KF.open(filename_before_KF.c_str());
//    f_before_KF << std::fixed;

//    cv::Mat T = cv::Mat::zeros(4, 4, CV_32F);
//    mSim3.rowRange(0,3).col(3).copyTo(T.rowRange(0,3).col(3));

    cv::Mat Tlc = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat eigt = mSim3.rowRange(0,3).col(3);
    eigt *= 1./mScale;
    mSim3.rowRange(0,3).colRange(0,3).copyTo(Tlc.rowRange(0,3).colRange(0,3));
    eigt.copyTo(Tlc.rowRange(0,3).col(3));

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
//        cv::Mat pkFi_pose = pKFi->GetPose();
//        f_before_KF << "Before: " << endl;
//        f_before_KF << pkFi_pose << endl;

//        cv::Mat s_ = pKFi->GetCameraCenter(); // Ow original
//        f_before_KF << s_.at<float>(0) << " " << s_.at<float>(1) << " " << s_.at<float>(2)  << "-----------";

//        f_before_KF << "Before: " << pKFi->GetPose() << endl;


//        cv::Mat R = pKFi->GetRotation() * mSim3.rowRange(0,3).colRange(0,3) ;
//        R.copyTo(Tcw.rowRange(0,3).colRange(0,3));
//        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
//        t = pKFi->GetRotation() * Translation + pKFi->GetCameraCenter();
//        t.copyTo(Tcw.rowRange(0,3).col(3)); //41.1793,37.0325,-36.3582

        cv::Mat Tcw_tem = pKFi->GetPose();
        cv::Mat pose =  Tcw_tem * Tlc ;

//        f_before_KF << "After: " << endl;
//        f_before_KF << pose << endl;

        pKFi->SetPose(pose);
//        f_before_KF << "After: " << pKFi->GetPose() << endl;

//        cv::Mat s__ = pKFi->GetCameraCenter(); // Ow now
//        f_before_KF << s__.at<float>(0) << " " << s__.at<float>(1) << " " << s__.at<float>(2)  << std::endl;

//        pKFi->ComputeBoW();
        pKFDB->add(pKFi);

    }

//    f_before_KF.close();

    cout << "End to rebuild KeyFrame references" << endl;
//-----------------------------------------------------------------------
//    std::string filename_before = "./log/MP_before.txt";
//
//    std::cout << std::endl << "Saving MP_before to txt ..." << std::endl;
//
//    std::ofstream f_before;
//    f_before.open(filename_before.c_str());
//    f_before << std::fixed;


    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Post-Load of mappoint " << pMPi->mnId << endl;
        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
        cv::Mat pMP_pose = pMPi->GetWorldPos();
//        f_before << pMP_pose.at<float>(0) << " " << pMP_pose.at<float>(1) << " " << pMP_pose.at<float>(2) << "---------------------";
        pMPi->SetWorldPos(mSim3.rowRange(0,3).colRange(0,3) * pMP_pose + mSim3.rowRange(0,3).col(3));
//        cv::Mat pMP_pose_ = pMPi->GetWorldPos();
//        f_before << pMP_pose_.at<float>(0) << " " << pMP_pose_.at<float>(1) << " " << pMP_pose_.at<float>(2) << endl;
        pMPi->UpdateNormalAndDepth(); // 更新观测 <
    }

//    f_before.close();

//    cout << "End to rebuild MapPoint references" << endl;
//
//
//    std::string filename = "./log/MP_load1.txt";
//
//    std::cout << std::endl << "Saving MP_load to txt ..." << std::endl;
//
//    std::ofstream f;
//    f.open(filename.c_str());
//    f << std::fixed;
//
//    for(MapPoint* pMPi : mspMapPoints){
//        cv::Mat t = pMPi->GetWorldPos();
//        f << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << endl;
//    }
//
//    f.close();
//--------------------------------------------------------------------
//    std::string filename_KF = "./log/KF.txt";
//
//    std::cout << std::endl << "Saving KFs to txt ..." << std::endl;
//
//    std::ofstream k;
//    k.open(filename_KF.c_str());
//    k << std::fixed;
//
//    for(KeyFrame* pKFi : mspKeyFrames){
//        cv::Mat R = pKFi->GetRotation().t();
//        vector<float> q = Converter::toQuaternion(R);
//        cv::Mat s = pKFi->GetCameraCenter();
////        k << s.at<float>(0) << " " << s.at<float>(1) << " " << s.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
//        k << s.at<float>(0) << " " << s.at<float>(1) << " " << s.at<float>(2)  << endl;
//
//    }
//
//    k.close();


    mvpBackupMapPoints.clear();
}

bool Map::ParseCamParamFile(cv::FileStorage &fSettings){

//    cv::Mat t = cv::Mat::eye(3,1, CV_32F);

    mSim3 = cv::Mat::eye(4, 4, CV_32F);
    Translation = cv::Mat::zeros(3, 1, CV_32F);

    cv::FileNode node = fSettings["Camera.sim30"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(0, 0) =node;
    }

    node = fSettings["Camera.sim31"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(0, 1) =node;
    }

    node = fSettings["Camera.sim32"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(0, 2) =node;
    }

    node = fSettings["Camera.sim33"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(0, 3) =node;
    }

    node = fSettings["Camera.sim34"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(1, 0) =node;
    }

    node = fSettings["Camera.sim35"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(1, 1) =node;
    }

    node = fSettings["Camera.sim36"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(1, 2) =node;
    }

    node = fSettings["Camera.sim37"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(1, 3) =node;
    }

    node = fSettings["Camera.sim38"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(2, 0) =node;
    }

    node = fSettings["Camera.sim39"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(2, 1) =node;
    }

    node = fSettings["Camera.sim310"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(2, 2) =node;
    }

    node = fSettings["Camera.sim311"];
    if(!node.empty() && node.isReal())
    {
        mSim3.at<float>(2, 3) =node;
    }

    node = fSettings["Camera.trans0"];
    if(!node.empty() && node.isReal())
    {
        Translation.at<float>(0,0) =node;
    }

    node = fSettings["Camera.trans1"];
    if(!node.empty() && node.isReal())
    {
        Translation.at<float>(1,0) =node;
    }

    node = fSettings["Camera.trans2"];
    if(!node.empty() && node.isReal())
    {
        Translation.at<float>(2,0) =node;
    }

    node = fSettings["Camera.scale"];
    if(!node.empty() && node.isReal())
    {
        mScale =node;
    }


    cout << "-----------------sim3------------------" << endl;
    cout << "sim3: " << mSim3 << endl;

    cout << "-----------------trans------------------" << endl;
    cout << "Translation: " << Translation << endl;

    cout << "-----------------scale------------------" << endl;
    cout << "Scale: " << mScale << endl;

//    cv::Mat Rcw = mSim3.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = mSim3.rowRange(0,3).col(3);
//    cv::Mat Rwc = Rcw.t();
//    cv::Mat Ow = -Rwc*tcw;

//    mSim3 = mSim3.inv();
//    Rwc.copyTo(mSim3.rowRange(0,3).colRange(0,3));
//    Ow.copyTo(mSim3.rowRange(0,3).col(3));

//    cout << "-----------------sim3------------------" << endl;
//    cout << "sim3: " << mSim3 << endl;
}
//void Map::PreSave(std::set<GeometricCamera*> &spCams)
//{   // 删除坏点
//    // 没有观测的地图点
//    int nMPWithoutObs = 0;
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        if(pMPi->GetObservations().size() == 0)
//        {
//            nMPWithoutObs++;
//        }
//        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
//        // 遍历所有地图 选出当前地图的地图点 删除不是在当前地图的地图点
//        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
//        {
//            // 当前帧的地图不是当前地图
//            if(it->first->GetMap() != this)
//            {
//                pMPi->EraseObservation(it->first); //We need to find where the KF is set as Bad but the observation is not removed
//            }
//
//        }
//    }
//    cout << "  Bad MapPoints removed" << endl;
//
//    // Saves the id of KF origins
//    // 保存最初始的关键帧
//    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
//    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
//    {
//        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
//    }
//
//    mvpBackupMapPoints.clear(); // 后面没用
//    // Backup of set container to vector
//    //std::copy(mspMapPoints.begin(), mspMapPoints.end(), std::back_inserter(mvpBackupMapPoints));
//
//    // 保存地图点
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        //cout << "Pre-save of mappoint " << pMPi->mnId << endl;
//        //mvpBackupMapPoints.push_back(pMPi);
//        pMPi->PreSave(mspKeyFrames,mspMapPoints);
//    }
//    cout << "  MapPoints back up done!!" << endl;
//
//    mvpBackupKeyFrames.clear(); // 后面没用
//    //std::copy(mspKeyFrames.begin(), mspKeyFrames.end(), std::back_inserter(mvpBackupKeyFrames));
//
//    // 保存关键帧
//    for(KeyFrame* pKFi : mspKeyFrames)
//    {
//        //mvpBackupKeyFrames.push_back(pKFi);
//        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
//    }
//    cout << "  KeyFrames back up done!!" << endl;
//
//    mnBackupKFinitialID = -1;
//    if(mpKFinitial)
//    {
//        mnBackupKFinitialID = mpKFinitial->mnId;
//    }
//
//    mnBackupKFlowerID = -1;
//    if(mpKFlowerID)
//    {
//        mnBackupKFlowerID = mpKFlowerID->mnId;
//    }
//
//}
//
//void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams)
//{
//    /*
//    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
//    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));
//    */
//
//    // 设置关键帧的字典
//    map<long unsigned int,MapPoint*> mpMapPointId;
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        pMPi->UpdateMap(this);
//        mpMapPointId[pMPi->mnId] = pMPi;
//    }
//
//    //map<long unsigned int, KeyFrame*> mpKeyFrameId;
//    for(KeyFrame* pKFi : mspKeyFrames)
//    {
//        pKFi->UpdateMap(this);
//        pKFi->SetORBVocabulary(pORBVoc);
//        pKFi->SetKeyFrameDatabase(pKFDB);
//        mpKeyFrameId[pKFi->mnId] = pKFi;
//    }
//
//    cout << "Number of KF: " << mspKeyFrames.size() << endl;
//    cout << "Number of MP: " << mspMapPoints.size() << endl;
//
//    // References reconstruction between different instances
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        //cout << "Post-Load of mappoint " << pMPi->mnId << endl;
//        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
//    }
//    cout << "End to rebuild MapPoint references" << endl;
//
//    for(KeyFrame* pKFi : mspKeyFrames)
//    {
//        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
//        pKFDB->add(pKFi);
//    }
//
//    cout << "End to rebuild KeyFrame references" << endl;
//
//    mvpBackupMapPoints.clear();
//}

} //namespace ORB_SLAM3
