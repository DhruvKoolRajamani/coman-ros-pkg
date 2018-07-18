template <typename S>
void StackAsVector(std::vector<S> &vec, const S &el, const unsigned int &SIZE)
{
    vec.push_back(el);
    if (vec.size() > SIZE){
        vec.erase(vec.begin());
    }
}

