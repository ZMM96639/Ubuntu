#include <bits/stdc++.h>

using namespace std;

void swap(int *xp, int *yp)
{
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// implement
void bubblesort(int arr[], int n)
{
    for (int i = 0; i < n - 1; ++i)
    {
        for (int j = 0; j < n - i - 1; ++j)
        {
            if (arr[j] > arr[j + 1])
            {
                swap(&arr[j], &arr[j + 1]);
            }
        }
    }
}

int main()
{
    int arr[] = {8, 9, 1, 4, 2, 3, 6, 7, 5};
    int n = sizeof(arr) / sizeof(arr[0]);
    bubblesort(arr, n);

    for (auto &&val : arr)
    {
        cout << val << ", ";
    }
    cout << endl;

    return 0;
}