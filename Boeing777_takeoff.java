import java.awt.event.*;
import java.awt.GraphicsConfiguration;
import com.sun.j3d.utils.applet.MainFrame;
import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.image.TextureLoader;
import com.sun.j3d.utils.universe.*;
import com.sun.j3d.loaders.IncorrectFormatException;
import com.sun.j3d.loaders.ParsingErrorException;
import com.sun.j3d.loaders.Scene;
import com.sun.j3d.loaders.objectfile.ObjectFile;
import javax.media.j3d.*;
import javax.vecmath.*; 
import java.lang.Object;
import java.applet.Applet;
import java.awt.BorderLayout;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Vector;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.GraphicsConfiguration;
import java.lang.System;


public class Boeing777_takeoff extends JApplet implements ActionListener
{
	//Time:
	long startTime = System.currentTimeMillis();
	long clickTime = 0;
	long waitTime = 10000; // 10 seconds wait time, laggbubbe
	
	//Constants:
	private float m;	
	private float g = 9.82f;                                             // gravity force
	private int A = 428;                                                 // wing area (both wings)
	private float CL = 0.55f;                                            // lift coefficient
	private float rho = 1.2f;                                            // air density
    private float Cd = 0.1f;                                           	 // drag coefficient
	private float theta = (float)Math.toRadians(0); 					 // angle of attack
	private float x = 0.0f;                                              // x-position
	private float y = 0.0f;                                              // y-position
	private float vx = 0.0f;                                             // horizontal velocity
	private float vxto;                                         		 // take-off velocity
	private float vy = 0.0f;                                             // vertical velocity   
	private float V = 0.0f;   											 // Airspeed V (vector direction velocity used in L and D)
	private float ax = 0.0f;                                             // horizontal acceleration    
	private float ay = 0.0f;                                             // vertical acceleration
	private float dt = 0.05f;                                            // time step
	private float timeto = 60.0f*dt;                                     // simulation duration	
           
	//Forces:
	private float T = 0.0f;                                              // Thrust (set manually)
	private float Tmax;
	private float W = 0.0f;                                              // Weight
    private float L = 0.0f;                                              // Lift
	private float D = 0.0f;                                              // Drag
		
	//GUI variables
	private JLabel lmass;
	private JLabel lthrust;
	private JLabel lvto;	
    private JLabel showvx;
    private JLabel showvy;
    private JLabel showx;
    private JLabel showy;
    private JLabel legendvx;
    private JLabel legendvy;
    private JLabel legendx;
    private JLabel legendy;
    private JSlider	sthrust;
	private JSlider	smass;
	private JSlider svto; 					
	private JButton	takeoff;	
	//private JButton	cancel;	
	private JTextField thrusttf;
	private JTextField masstf;
	private JTextField vtotf;				
	private JPanel scenep; 	
	private JPanel prop;	
	private JPanel view;	
	
	//Vectors:	
	Vector<Float> yValues = new Vector<Float>();
	Vector<Float> xValues = new Vector<Float>();
	Vector<Float> vxValues = new Vector<Float>();
	Vector<Float> vyValues = new Vector<Float>();
		
	//Physics:
	public void physicsCalculations()
	{
		
		m = (float)smass.getValue()*1000;
		Tmax = (float)sthrust.getValue()*1000;
		vxto = ((float)svto.getValue())/3.6f;
		W=m*g;
			
			for (float i=1; i<=(timeto/dt); i=i+dt)
			{
				if (T < Tmax)
				{
					T = T+Tmax/(6/dt); // T from 0 to full in 6 seconds
				} 
		  
				V = vx/((float)Math.cos(theta));
		  
				L = 0.5f*rho*((float)Math.pow(V,2))*A*CL;               
				D = 0.5f*rho*((float)Math.pow(V,2))*A*Cd; 

				ax = (T*((float)Math.cos(theta))-D*((float)Math.cos(theta))-L*((float)Math.sin(theta)))/m; 
				
				if (theta >= (float)Math.toRadians(14))
				{
					ax = 0;  // stops accelerating when 14 degrees angle reached
				}
				
				// Euler method to calculate the new x-values:
				vx = euler(vx,ax,dt);
				x = euler(x,vx,dt);

	
				if (vx >= vxto)
				{
					CL = 1.6f; // new lift coefficient
							    
				    if (theta < (float)Math.toRadians(9))
				    {
				    	theta=theta+((float)Math.toRadians(9))/(4.0f/dt); // from 0 to 9 degrees during 4 seconds
				    }
					        
				    else if  (theta >= (float)Math.toRadians(9) &&  theta < (float)Math.toRadians(14))
				    { 
				    	theta=theta+((float)Math.toRadians(5))/(2.5f/dt); // from 9 to 14 degrees during 2.5 seconds    
				    }

				      	
				    ay = (L*((float)Math.cos(theta))+T*((float)Math.sin(theta))-D*((float)Math.sin(theta))-W)/m;
				    if (ay < 0)
				    {
				    	ay = 0; // stops the airplane from going through the ground, if ay would get negative
				    } 
				    
				    if (theta >= (float)Math.toRadians(14))
				    {
				    	ay=0; // stops accelerating when 14 degrees angle reached
				    }
				    
				    // Euler method to calculate the new y-values:
				    vy = euler(vy, ay, dt);
				    y = euler(y, vy,dt);
									        
				}
				    // Add values to vectors:
					xValues.add(x);
					yValues.add(y);
					vxValues.add(vx);
					vyValues.add(vy);						   
			}
			
			/*
			//just some code to check the results...
			float grader = (float)Math.toDegrees(theta);
			System.out.println("vtxo = " + vxto);
			System.out.println("m = " + m);
			System.out.println("y = " + y);
			System.out.println("vx = " + vx);
			System.out.println("vy = " + vy);
			System.out.println("x = " + x);
			System.out.println("ay = " + ay);
			System.out.println("T = " + T);
			System.out.println("ax = " + ax);
		    System.out.println("grader = " + grader);
		    System.out.println("i = " + timeto/dt);
		    System.out.println("sin(theta) = " + (float)Math.sin(theta));
		    System.out.println("CL = " + CL);
		    System.out.println("size på y-vektorn: " + yValues.size());
			System.out.println("D = " + D);
			System.out.println("rho = " + rho);
			System.out.println("(float)Math.pow(V,2) = " + (float)Math.pow(V,2));
			System.out.println("A = " + A);
			System.out.println("Cd = " + Cd);
			System.out.println(T);
			*/		   
		}
			
		public float euler(float a, float b, float c)
		{  
	    	float xprim = (a+b*c);
		    return xprim;
		}
	
		
	private SimpleUniverse universe = null;
    public BranchGroup createSceneGraph() 
    {    	
		// Create the root of the branch graph
		BranchGroup objRoot = new BranchGroup();
		
		// Load the models
		Scene theAircraft = loadModel("777.obj");
		Scene theTerrain = loadModel("enviroment.obj");	
		
	  	Transform3D viewtransform = new Transform3D(); // Global view transformation
	    viewtransform.rotX((float)Math.PI/27.0f);
	    Vector3f viewPos = new Vector3f(0.0f, 0.0f, 0.0f);
		viewtransform.setTranslation(viewPos);
		
       	Transform3D translation = new Transform3D();
	   	Vector3f pos = new Vector3f(0.0f, 0.0f, 0.0f);
		translation.setTranslation(pos);
		
		Transform3D yTranslation = new Transform3D();
		AxisAngle4f bar = new AxisAngle4f(0.0f, 0.0f, 1.0f, (float)Math.toRadians(90)); // used to get y- instead of x-translation
		yTranslation.set(bar);
		
		Transform3D xTranslation = new Transform3D();
		Vector3f bar2 = new Vector3f(0.0f, 0.0f, 0.0f);
		xTranslation.set(bar2);
		
		Transform3D terrainTranslation = new Transform3D(); // used for the animation
		
		Transform3D terrainTranslation2 = new Transform3D(); // used for initial positioning
	   	Vector3f tpos2 = new Vector3f(-6.2f, 1.71f, -2.477f);
		terrainTranslation2.setTranslation(tpos2);	   
	
		Transform3D scale = new Transform3D();
		scale.setScale(15.0f);
		Transform3D scale2 = new Transform3D();
		scale2.setScale(50.0f);	
		Transform3D scale3 = new Transform3D();
		scale3.setScale(0.008f);
					
		// Transformgroups
		TransformGroup viewRotate = new TransformGroup(viewtransform);
		TransformGroup objRotate = new TransformGroup();
		TransformGroup objScale = new TransformGroup(scale);
		TransformGroup objScale2 = new TransformGroup(scale2);
		TransformGroup objScale3 = new TransformGroup(scale3);
		TransformGroup objTranslate = new TransformGroup(translation);
		TransformGroup objTranslate2 = new TransformGroup(terrainTranslation2);
		TransformGroup objTerrainTranslate = new TransformGroup(terrainTranslation);
					
		objRotate.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		objTranslate.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		objTerrainTranslate.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		
	    objRoot.addChild(viewRotate);
		viewRotate.addChild(objScale3); 
	    objScale3.addChild(objTranslate);
		objTranslate.addChild(objRotate);
		objRotate.addChild(objScale);	
		objScale.addChild(theAircraft.getSceneGroup()); // Add aircraft
		
		viewRotate.addChild(objTranslate2); 
		objTranslate2.addChild(objTerrainTranslate);
		objTerrainTranslate.addChild(objScale2);
		objScale2.addChild(theTerrain.getSceneGroup()); // Add terrain
			 	
    	BoundingSphere bounds = new BoundingSphere(new Point3d(0.0,0.0,0.0), 100.0); 
		 						
	   	// Set up the background
		Color3f bgColor = new Color3f(0.0f, 0.0f, 0.2f);
		Background bgNode = new Background(bgColor);
		bgNode.setApplicationBounds(bounds);
		objRoot.addChild(bgNode);
	    
	    for (int i = 0; i < xValues.size()-1; i++)        	
	    {
       	   	Alpha alpha = new Alpha(1, (clickTime-startTime+waitTime+(i-1)*51), 0, 51, 0, 0);    
	  	   	PositionInterpolator xTranslator = new PositionInterpolator(alpha, objTerrainTranslate, terrainTranslation, xValues.get(i)/100f, xValues.get(i+1)/100f);
			PositionInterpolator yTranslator = new PositionInterpolator(alpha, objTranslate, yTranslation, yValues.get(i), yValues.get(i+1));
			xTranslator.setSchedulingBounds(bounds);
			yTranslator.setSchedulingBounds(bounds);
			objTerrainTranslate.addChild(xTranslator);
			objTranslate.addChild(yTranslator);
			
			Transform3D rotateY = new Transform3D();
	  		rotateY.rotX(-Math.PI / 2.0); 
			Alpha rotAlpha1 = new Alpha(1, clickTime-startTime+waitTime+(i-1)*51, 0, 4000, 0, 0); // 1 loop, starts after 4000 ms, 4000 ms cycle
			Alpha rotAlpha2 = new Alpha(1, clickTime-startTime+waitTime+4000+(i-1)*51, 0, 2500, 0, 0); // 1 loop, starts after the first loop, 2500 ms cycle

			if (vxValues.get(i)>=vxto) // when take-off speed reached, do this:
			{
				RotationInterpolator rotator1 = new RotationInterpolator(rotAlpha1, objRotate, rotateY,
				0.0f, (float)(Math.toRadians(9.0d))); //0 to 9 degrees
				RotationInterpolator rotator2 =  new RotationInterpolator(rotAlpha2, objRotate, rotateY,
				(float)(Math.toRadians(9.0d)), (float)(Math.toRadians(14.0d))); //9 to 14 degrees
				rotator1.setSchedulingBounds(bounds);
				rotator2.setSchedulingBounds(bounds);		
				objRotate.addChild(rotator1);
				objRotate.addChild(rotator2);
			}			
		
	    }
	    
	    objRoot.compile();    
		return objRoot;		
	}
    
    public void init() 
    {    	    	
	    setLayout(new BorderLayout());
	    GraphicsConfiguration config = SimpleUniverse.getPreferredConfiguration();
		Canvas3D bild = new Canvas3D(config);
		add("Center", bild);
	  	
		universe = new SimpleUniverse(bild);	  
	    universe.getViewingPlatform().setNominalViewingTransform();
		 				
		ViewingPlatform viewingPlatform = universe.getViewingPlatform();
		PlatformGeometry pg = new PlatformGeometry();
		
	    BoundingSphere bounds = new BoundingSphere(new Point3d(0.0, 0.0, 0.0), 100.0); 
	
		//Set up lighting
	  	Color3f light1Color = new Color3f(1.0f, 1.0f, 0.7f);
	    Vector3f light1Direction = new Vector3f(8.0f, -20.0f, -17.0f);

   	    DirectionalLight light1 = new DirectionalLight(light1Color, light1Direction);

   		light1.setInfluencingBounds(bounds);
		pg.addChild(light1);
		viewingPlatform.setPlatformGeometry(pg);
		

	        
		
		//GUI	
		lmass = new JLabel("Mass (tons):");
		lthrust = new JLabel("Max thrust (kN):");
		lvto = new JLabel("Take-off speed (km/h):");
		
	    legendvx = new JLabel("x velocity (km/h): ");
   		legendvy = new JLabel("y velocity (km/h): ");
 	    legendx = new JLabel("Distance (m): ");
        legendy = new JLabel("Height (m): ");
	    showvx = new JLabel("0");
    	showvy = new JLabel("0");
     	showx = new JLabel("0");
    	showy = new JLabel("0");
    				
  		sthrust = new JSlider(500, 1020, 1020);
		smass = new JSlider(200, 300, 280);
		svto = new JSlider(270, 345, 290);
	
		takeoff = new JButton("Take Off");
		//cancel = new JButton("Restart");		
	    thrusttf = new JTextField("1020" ,4);
		masstf = new JTextField("280", 3);
		vtotf = new JTextField("290", 3);				
		scenep = new JPanel(new BorderLayout()); 	//Scene JPanel
		prop = new JPanel(new FlowLayout());		//Properties JPanel
		view = new JPanel(new BorderLayout());		//View Selection JPanel
		
		//lägg i JPanel prop
		prop.add(lmass);
		prop.add(smass);
		prop.add(masstf);
		masstf.addActionListener(this);
		
		prop.add(lthrust);
		prop.add(sthrust);
		prop.add(thrusttf);
		thrusttf.addActionListener(this);
		
		prop.add(lvto);
		prop.add(svto);
		prop.add(vtotf);
		
		vtotf.addActionListener(this);
				
		takeoff.addActionListener(this);
		//cancel.addActionListener(this);
		
		smass.addChangeListener(changeListener);
		sthrust.addChangeListener(changeListener2);
		svto.addChangeListener(changeListener3);
		
		//lägg i knappar för view i JPanel view
		//view.add(cancel, BorderLayout.WEST);
		view.add(takeoff, BorderLayout.EAST);
		
		scenep.add(bild, BorderLayout.CENTER);
		scenep.add(prop, BorderLayout.NORTH);
		scenep.add(view, BorderLayout.SOUTH);
	
		//container for JPanel
		Container c = getContentPane();
		c.setLayout(new BorderLayout());
		c.add(scenep, BorderLayout.CENTER);
		c.add(prop, BorderLayout.NORTH);
		c.add(view, BorderLayout.SOUTH);	
    }

    /*public void destroy() 
    {
		universe.removeAllLocales();
    }*/
   
    // loader
   	public Scene loadModel(String filename)
   	{
		Scene loadedScene = null;
		ObjectFile f = new ObjectFile(ObjectFile.RESIZE); 
		try {
			loadedScene = f.load(filename);
		}
			catch (FileNotFoundException e) {
				System.err.println(e);
				System.exit(1);
		}
			catch (ParsingErrorException e) {
				System.err.println(e);
				System.exit(1);
		}
			catch (IncorrectFormatException e) {
				System.err.println(e);
				System.exit(1);
		}
			return loadedScene;
   	}
   	   	
   		ChangeListener changeListener = new ChangeListener() {
      	public void stateChanged(ChangeEvent changeEvent) {
        	JSlider smass = (JSlider)changeEvent.getSource();  // get the slider	
     			if (smass.getValueIsAdjusting()) {
       			m = smass.getValue();  // get slider value
        		masstf.setText("" + ((int)Math.round(smass.getValue() )));
        	    }
     			
     			
      }
    };
    	ChangeListener changeListener2 = new ChangeListener() {
    		public void stateChanged(ChangeEvent changeEvent2){
    			JSlider sthrust = (JSlider)changeEvent2.getSource();
     			if(sthrust.getValueIsAdjusting()){
     				Tmax = sthrust.getValue();
     				thrusttf.setText("" + ((int)Math.round(sthrust.getValue() )));
     			}
    		}
    	};
    	
    	ChangeListener changeListener3 = new ChangeListener() {
    		public void stateChanged(ChangeEvent changeEvent3){
    			JSlider svto = (JSlider)changeEvent3.getSource();
     			if(svto.getValueIsAdjusting()){
     				vxto = svto.getValue();
     				vtotf.setText("" + ((int)Math.round(svto.getValue() )));
     			}
    		}
    	};
    	 	
  
    	     	
    public void actionPerformed(ActionEvent e){
    int slidevalue = Integer.parseInt(masstf.getText());
        smass.setValue(slidevalue); 
        	
	int slidevalue2 = Integer.parseInt(thrusttf.getText());
		sthrust.setValue(slidevalue2);
		
    int slidevalue3 = Integer.parseInt(vtotf.getText());
		svto.setValue(slidevalue3);
		
	if(e.getSource() == takeoff)
	{
		clickTime = System.currentTimeMillis();
	  	physicsCalculations();
	    BranchGroup scene = createSceneGraph();
		universe.addBranchGraph(scene);
		sthrust.setEnabled(false);
		smass.setEnabled(false);
		svto.setEnabled(false);
		lthrust.setEnabled(false);
		lmass.setEnabled(false);
		lvto.setEnabled(false);
		thrusttf.setEnabled(false);
		masstf.setEnabled(false);
		vtotf.setEnabled(false);
		takeoff.setEnabled(false);       
	}
	/*
	if(e.getSource() == cancel)
	{
	//	destroy();
	}
	*/	    	
    }   	
   	   	 	
    public static void main(String[] args) 
    {
		new MainFrame(new Boeing777_takeoff(), 1200, 800);
    }
}
