void WhichSide(const double &k,const double &s, unsigned int &side, int &sg)
{
#ifdef REAL_ROBOT
        if (k > 0.55 && s > 0) //TEMP ? 0.1 or less
        {
            side = 1;
            sg = 1;
        }
        else if (k <=0.45 && s > 0)
        {
            side = 0;
            sg = -1;
        }
#else
        if (k > 0.55 && s > 0)
        {
            side = 1;
            sg = 1;
        }
        else if (k <= 0.45 && s > 0)
        {
            side = 0;
            sg = -1;
        }
#endif

}
