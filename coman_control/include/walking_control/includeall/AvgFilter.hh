template <class S>

void AvgFilter(std::vector<S> &vec, S &avgOfVec, const S &el, const unsigned int &SIZE)
{
    vec.push_back(el);
    if (vec.size() <= SIZE){
         avgOfVec = (avgOfVec * (vec.size() - 1) + el) / vec.size();
    }
    else{
        avgOfVec = ((avgOfVec * (SIZE))-vec.front() + el) / SIZE;
        vec.erase(vec.begin());
    }
}
