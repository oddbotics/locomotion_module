#ifndef MODULE_H_
#define MODULE_H_

class Module 
{
  public:
    Module(int port, std::sting type): port(port) {}
    std::string getPort(){return this->port;}
    void setLocation(std::string port){this->port = port;}
    virtual std::string getType() = 0; 

  protected:
    
    std::string port; 

};

#endif // MODULE_H_
