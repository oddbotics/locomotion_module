#ifndef MODULE_H_
#define MODULE_H_

class Module 
{
  public:
    Module(int port): port(port) {}
    int getPort(){return this->port;}
    virtual std::string getType() = 0; 

  protected:
    
    int port; 

};

#endif // MODULE_H_
