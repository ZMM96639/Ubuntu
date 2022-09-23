### Using Eigen

##### 1. Camera Imaging Model

$$
\begin{cases}
u = f_x\frac{X_c}{Z_c}+u_0\\
v = f_y\frac{Y_c}{Z_c}+v_0
\end{cases},
$$ $$
s\begin{bmatrix}
u \\ v \\ 1
\end{bmatrix}=
\begin{bmatrix}
f_x & 0 & u_0 \\
0 & f_y & v_0\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X_c \\ Y_c \\ Z_c
\end{bmatrix}=
\begin{bmatrix}
f_x & 0 & u_0\\
0 & f_y & v_0\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
R_w^c & t_w^c
\end{bmatrix}
\begin{bmatrix}
X_w \\ Y_w \\ Z_w \\ 1
\end{bmatrix}.
$$
    
    Class CamMod {
    public:
        CamMod() = default;
        CamMod(const Eigen::Matrix3d &K,
               const Eigen::Matrix3d &rotation,
               const Vector3d &translation,
               const Eigen::Vector3d &wp_pt):m_K{K}, m_rotation{rotation}, m_translation{translation}, m_wp_pt{wp_pt} {}
        ~CamMod() = default;

    private:
        CamMod(const CamMode &) = delete;
        CamMod &operator=(const CamMod &) = delete;

        Eigen::Isometry3d transform(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
            Eigen Isometry3d pose{Eigen::Isometry3d::Identity()};
            pose.rotate(rotation);
            pose.pretranslate(translation);
            return pose;
        }

    public:
        void computeImage_pt() {
            Eigen::Isometry3d trans_to_cam = transform(this->m_rotation, this->m_translation);
            Eigen::Vector3d cam_pt = trans_to_cam * this->m_wp_pt;
            this->m_img_pt = (m_K * cam_pt) / cam_pt[2];
        }

    private:
        Eigen::Matrix3d m_K{Eigen::Matrix3d::Identity()};

        Eigen::Matrix3d m_rotation{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d m_translation{Eigen::Vector3d::Zero()};

        Eigen::Vector3d m_wp_pt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d m_img_pt{Eigen::Vector2d::Zero()};
    };
    

##### 2. Calculating $\sqrt x$ using the optimization method
    
    template <class T>
    class Mysqrt
    {
    private:
        mutable T m_x{};

    public:
        Mysqrt() = default;

        explicit Mysqrt(const T &x) : m_x{x}
        {
            std::cout << "contruction is recalled!" << std::endl;
        }

        Mysqrt(const Mysqrt &&rhs) : m_x{rhs.m_x}
        {
            std::cout << "move construction is recalled!" << std::endl;
        }

        Mysqrt &operator=(const Mysqrt &&rhs)
        {
            std::cout << "move assignment is recalled!" << std::endl;
            m_x = rhs.m_x;
            return *this;
        }

        ~Mysqrt()
        {
            std::cout << "destrution is recalled!" << std::endl;
        }

    private:
        Mysqrt(const Mysqrt &) = delete;
        Mysqrt &operator=(const Mysqrt &) = delete;

        T implement(const T &x) const
        {
            if (x == 0)
            {
                return 0;
            }

            T x0 = x;
            while (1)
            {
                T xi = 0.5 * (x0 + x / x0);
                if (fabs(x0 - xi) < 1e-7)
                {
                    break;
                }
                x0 = xi;
            }
            return x0;
        }

    public:
        T getVal()
        {
            T tmp = implement(m_x);
            return tmp;
        }
    };