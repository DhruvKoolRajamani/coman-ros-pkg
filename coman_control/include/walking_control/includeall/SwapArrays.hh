template <typename T>
void SwapArrays(T firstArray, T secondArray, unsigned int size)
{
    double tempVar;
    for (int i = 0; i < size; i++)
    {
        tempVar = firstArray[i];
        firstArray[i] = secondArray[i];
        secondArray[i] = tempVar;
    }
}
