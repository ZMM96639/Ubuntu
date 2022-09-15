#include <opencv2/highgui/highgui.hpp>

using namespace std;

class Mouse
{
public:
    Mouse();

    ~Mouse();

    static void onMouse(int event, int x, int y, int flags, void *param);

    static vector<cv::Point> ppoints;

private:
    
};
