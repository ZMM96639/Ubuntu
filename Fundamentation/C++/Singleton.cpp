template <class T>
class Singleton{
private:
    static T* m_instance;

public:
    Singleton(){
        assert(m_instance == nullptr);
        m_instance = static_cast<T*>(this);
    }

    virtual ~Singleton(){
        m_instance = nullptr;
    }

    static T& getSingleton(){
        return *m_instance;
    }

    static T* getSingletonPtr(){
        return m_instance;
    }
};
