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


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/set.hpp>

#include<mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        //ar & mpVoc;// Cannot be serialized (DBoW2 object)
        ar & mvInvertedFile;//Required for KeyFrame References
        ar & mvBackupInvertedFileId;
    }

public:

//    KeyFrameDatabase(const ORBVocabulary &voc);

   KeyFrameDatabase();

   KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();
   void clearMap(Map* pMap);

   // Loop Detection(DEPRECATED)
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Loop and Merge Detection
   void DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand);
   void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
   void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

   void PreSave();
   void PostLoad(map<long unsigned int, KeyFrame*> mpKFid);

   void SetORBVocabulary(ORBVocabulary* pORBVoc);
   void SetORBVocabularyPostLoad(const ORBVocabulary &voc);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  // 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  // 保存了关键帧的id
  std::vector<list<long unsigned int> > mvBackupInvertedFileId;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
