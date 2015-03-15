typedef struct 
{
  int A;
  int B;
  long result;
} factors_t;


void setup() 
{  
  Serial.begin(9600); 
}


// function prototype, important when passing typedefs
// - and please notice that I'm passing by reference, to
// get result out with us afterwards
long multiply(factors_t &f);

long multiply(factors_t &f)
{
  f.result = (long)f.A * f.B;
}

void loop()
{
  factors_t X;
  X.A = 123;
  X.B = 321;
  multiply(X);


  Serial.println(X.result);
  delay(500);
}
