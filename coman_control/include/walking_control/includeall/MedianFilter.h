#include <algorithm>
template <typename S>
void MedianFilter(std::vector<S> &vec, S &medOfVec,const S &el, const unsigned int &SIZE)
{
    if (vec.size() <= SIZE){
        vec.push_back(el);
        std::vector<S> tempVec = vec;
        std::sort(tempVec.begin(), tempVec.end());
        medOfVec = tempVec[vec.size() / 2];
    }
    else{
        vec.push_back(el);
        vec.erase(vec.begin());
        std::vector<S> tempVec = vec;
        std::sort(tempVec.begin(), tempVec.end());
        medOfVec = tempVec[SIZE / 2];
    }
}
