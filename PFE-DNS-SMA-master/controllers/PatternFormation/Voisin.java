// import Math;

public class Voisin{
    private String name;
    private double x;
    private double y;
    private double theta;
    
    public Voisin(String name, double x, double y, double theta){
           this.name = name;
           this.x = x;
           this.y = y;
           this.theta = theta;
    }
    
    public Voisin(){
           this.name = "name";
           this.x = 0;
           this.y = 0;
           this.theta = 0;
    }
    // gatters and setters
    public String get_name(){
           return this.name;
           }
    public double get_x(){
           return this.x;
           }
           
     public double get_y(){
           return this.y;
           }
      
      public double get_theta(){
           return this.theta;
           }
           
      public void set_theta(double theta){
           this.theta = theta;
           }
           
     public void set_y(double y){
           this.y = y;
           }
      
      public void set_x(double x){
           this.x = x;
           }
      public void set_name(String name){
           this.name = name;
           }
      // Calculent le sin et cos de l'angle entre le vecteur Ri et le vecteur RRvoisin
      public double cos_angle(double x, double y){   
            return (this.x - x)/distance(x, y);
      }
      
      public double sin_angle(double x, double y){    
            return (this.y - y)/distance(x, y);
      }
      
      public void afficher(){
          System.out.println("Robot :{name:"+this.get_name()+",coord:("+this.get_x()+","+this.get_y()+")}");
      }
      
      public double distance(double x, double y){
      // Math.sqrt ((x1-x2)* (x1-x2) + (y1-y2)* (y1-y2))
             return Math.sqrt((x - this.x)*(x - this.x) + (y - this.y)*(y - this.y));
      }
}