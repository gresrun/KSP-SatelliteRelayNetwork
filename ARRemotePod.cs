using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

/*
 * Version 0.2:
 * 
 * Fixed flickering SAS
 * Fixed inefficient pathfinding
 * Allowed commandable vessels to be comsats even if they have "debris" in their names
 * Made some fields public so other code can access the comms state
 * 
 */

public class ARRemotePod : CommandPod
{
    public double speedOfLight = 30000000.0; //default speed of light 30,000 km/s; (one tenth real speed)
    public String comsatString = "comsat";
    public String basePlanet = "Kerbin";
    public double baseLatitude = -0.340;
    public double baseLongitude = 27.620;

    //you can use these public values to make your code react based on the comms state. for instance
    //you can write
    //
    //if(vessel.rootPart is ARRemotePod && ((ARRemotePod)vessel.rootPart).inRadioContact) 
    //{
    //    then the command pod is an ARRemotePod which is claiming to have radio contact with mission
    //    control through the comms network
    //}
    public bool inRadioContact = true; //whether the vessel currently has a relay path to mission control
    public RelayPath controlPath;      //current best relay path
    public double controlDelay = 0;    //round trip speed of light delay in seconds

    int ticksSinceContactCheck = 0;

    FlightCtrlStateBuffer delayedBuffer = new FlightCtrlStateBuffer();

    protected override void onPartFixedUpdate()
    {

        if (ticksSinceContactCheck++ > 100)
        {
            ticksSinceContactCheck = 0;

            controlPath = findShortestRelayPath();
            if (controlPath == null)
            {
                inRadioContact = false;
                print("ARRemotePod: no radio contact!!");
            }
            else
            {
                inRadioContact = true;
                controlDelay = 2 * controlPath.Length / speedOfLight;
                print("ARRemotePod: radio contact: " + controlPath.ToString());
                print("ARRemotePod: signal path length (km) = " + controlPath.Length / 1000.0 + "; round trip time (s) = " + controlDelay);
            }
        }



        base.onPartFixedUpdate();

    }

    //check whether any celestial bodies block line of sight between a and b.
    bool lineOfSight(Vector3d a, Vector3d b)
    {
        foreach (CelestialBody body in FlightGlobals.Bodies)
        {
            if (Vector3d.Dot(body.position - a, b - a) > 0)
            {
                if (Vector3d.Dot(body.position - a, (b - a).normalized) < (b - a).magnitude)
                {
                    //check lateral offsite from line between b and a
                    Vector3d lateralOffset = (body.position - a) - Vector3d.Dot(body.position - a, (b - a).normalized) * (b - a).normalized;
                    if (lateralOffset.magnitude < body.Radius - 5)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }


    RelayPath findShortestRelayPath()
    {
        Vector3d baseRelay = computeBaseRelayPosition();

        List<RelayPath> paths = new List<RelayPath>();
        paths.Add(new RelayPath(this.vessel));

        List<Vessel> routedVessels = new List<Vessel>();

        while (true)
        {
            //find shortest path in list
            RelayPath shortestPath = null;
            foreach (RelayPath path in paths)
            {
                if (shortestPath == null || path.Length < shortestPath.Length) shortestPath = path;
            }

            if (shortestPath == null) return null;

            if (shortestPath.Terminated) return shortestPath;

            paths.Remove(shortestPath);

            if (routedVessels.Contains(shortestPath.Endpoint))
            {
                print("ARRemotePod: pathing error?");
            }
            else
            {
                routedVessels.Add(shortestPath.Endpoint);
            }

            if (lineOfSight(shortestPath.Endpoint.transform.position, baseRelay))
            {
                RelayPath newPath = new RelayPath(shortestPath);
                newPath.terminate(baseRelay);
                paths.Add(newPath);
            }

            foreach (Vessel v in FlightGlobals.Vessels)
            {
                if (isComsat(v) && !routedVessels.Contains(v) && lineOfSight(v.transform.position, shortestPath.Endpoint.transform.position))
                {
                    RelayPath newPath = new RelayPath(shortestPath);
                    newPath.Add(v);
                    paths.Add(newPath);
                }
            }
        }
    }

    //decide whether a vessel is a comsat
    bool isComsat(Vessel v)
    {
        return (v.vesselName.ToLower().Contains(comsatString) && (v.isCommandable || !v.vesselName.ToLower().Contains("debris")));
    }


    //figure out where mission control is:
    Vector3d computeBaseRelayPosition()
    {
        //find base body:
        CelestialBody baseBody = null;
        foreach (CelestialBody body in FlightGlobals.Bodies)
        {
            if (body.name.ToLower().Contains(basePlanet.ToLower()))
            {
                baseBody = body;
                break;
            }
        }

        if (baseBody == null)
        {
            print("ARRemotePod: couldn't find base planet!!");
            return this.vessel.transform.position; //give up
        }

        return baseBody.position + baseBody.Radius * baseBody.GetSurfaceNVector(baseLatitude, baseLongitude);
    }

    bool myKillRot = false;
    bool lastKillRot = false;

    public void drive(FlightCtrlState s)
    {
        if (!inRadioContact)
        {
            //lock out the player if we are out of radio contact
            FlightInputHandler.SetNeutralControls();
        }
        else
        {
            //usually the game feeds back whatever value of killrot we gave it last frame. If this is not true, then the 
            //user toggled SAS.
            if (s.killRot != lastKillRot) 
            {
                myKillRot = !myKillRot;
            }
            s.killRot = myKillRot;
            delayedBuffer.push(s, Planetarium.GetUniversalTime());
            delayedBuffer.pop(s, Planetarium.GetUniversalTime() - controlDelay);
            lastKillRot = s.killRot;
        }

    }

    void drawGUI()
    {
        Color savedColor = GUI.color;
        if (inRadioContact && controlPath != null)
        {
            GUI.color = Color.yellow;
            GUI.Label(new Rect(125, 1, 400, 300), "Relay path: " + controlPath.ToString() + "\n"
                + String.Format("Path length: {0:0} km, round trip delay: {1:0.00} s", controlPath.Length / 1000.0, controlDelay));
        }
        else
        {
            GUI.color = Color.red;
            GUI.Label(new Rect(150, 10, 300, 40), "Out of radio contact!");
        }
        GUI.color = savedColor;
    }
    

    protected override void onPartAwake()
    {
        //make the vessel crewless
        GameObject go = GameObject.Find("internalSpace");
        if (go != null)
        {
            Transform t = go.transform;
            if (t != null)
            {
                Transform child = t.FindChild("mk1pod_internal");
                if (child != null)
                {
                    InternalModel im = child.GetComponent<InternalModel>();
                    if (im != null)
                    {
                        im.seats = new Transform[0];
                    }
                }
            }
        }

        base.onPartAwake();
    }

    protected override void onFlightStart()
    {
        print("ARRemotePod: speedOfLight = " + speedOfLight);
        print("ARRemotePod: comsatString = " + comsatString);
        print("ARRemotePod: basePlanet = " + basePlanet);
        print("ARRemotePod: baseLatitude = " + baseLatitude);
        print("ARRemotePod: baseLongitude = " + baseLongitude);



        FlightInputHandler.OnFlyByWire += new FlightInputHandler.FlightInputCallback(this.drive);
        RenderingManager.AddToPostDrawQueue(3, new Callback(drawGUI));


        base.onFlightStart();
    }

    protected override void onPartDestroy()
    {

        FlightInputHandler.OnFlyByWire -= new FlightInputHandler.FlightInputCallback(this.drive);
        RenderingManager.RemoveFromPostDrawQueue(3, new Callback(drawGUI)); //close the GUI

        base.onPartDestroy();
    }

    protected override void onDisconnect()
    {
        FlightInputHandler.OnFlyByWire -= new FlightInputHandler.FlightInputCallback(this.drive);
        RenderingManager.RemoveFromPostDrawQueue(3, new Callback(drawGUI)); //close the GUI

        base.onPartDestroy();
    }
}


public class RelayPath
{
    //vessels through which the signal is relayed
    public List<Vessel> relays = new List<Vessel>();
    
    //whether the relay path reaches mission control
    public bool terminated = false;

    //length of the relay leg the terminates at mission control
    //actually, we should really be storing the location of mission
    //control because this leg length will change as the comsat that
    //connects to mission control moves. so the relay path length
    //calculation isn't quite correct right now
    private double lastLegLength = 0;

    public RelayPath(Vessel origin)
    {
        relays.Add(origin);
    }

    public RelayPath(RelayPath rhs)
    {
        this.relays = new List<Vessel>(rhs.relays);
        this.terminated = rhs.terminated;
        this.lastLegLength = rhs.lastLegLength;
    }


    public void Add(Vessel v)
    {
        relays.Add(v);
    }

    public bool Contains(Vessel v)
    {
        return relays.Contains(v);
    }

    public void terminate(Vector3d position)
    {
        terminated = true;
        lastLegLength = (this.Endpoint.transform.position - position).magnitude;
    }

    public Vessel Endpoint
    {
        get
        {
            if (relays.Count == 0) return null;
            return relays[relays.Count - 1];
        }
    }

    public double Length
    {
        get
        {
            double length = 0;
            for (int i = 1; i < relays.Count; i++)
            {
                length += (relays[i].transform.position - relays[i - 1].transform.position).magnitude;
            }
            if (terminated) length += lastLegLength;
            return length;
        }
    }

    public bool Terminated
    {
        get
        {
            return terminated;
        }
    }

    public override String ToString()
    {
        if (relays.Count == 0) return "empty path????";
        String ret = "";
        if (terminated) ret += "Mission Control";

        for (int i = relays.Count - 1; i >= 0; i--)
        {
            ret += " → " + relays[i].vesselName;
        }
        return ret;
    }

    void print(String s)
    {
        MonoBehaviour.print(s);
    }
}

//used to delay ctrl inputs by saving FlightCtrlStates between the time they are entered
//by the user and the time they are applied to the craft
class FlightCtrlStateBuffer
{
    Queue<FlightCtrlState> states = new Queue<FlightCtrlState>();
    Queue<double> times = new Queue<double>();

    //save the flight control state entered at a given time
    public void push(FlightCtrlState state, double time)
    {
        FlightCtrlState savedCopy = new FlightCtrlState();
        copyFlightCtrlState(state, savedCopy);
        states.Enqueue(savedCopy);
        times.Enqueue(time);
    }

    void copyFlightCtrlState(FlightCtrlState s, FlightCtrlState copy)
    {
        copy.activate = s.activate;
        copy.fastThrottle = s.fastThrottle;
        copy.gearDown = s.gearDown;
        copy.gearUp = s.gearUp;
        copy.killRot = s.killRot;
        copy.mainThrottle = s.mainThrottle;
        copy.pitch = s.pitch;
        copy.rcs = s.rcs;
        copy.roll = s.roll;
        copy.switchCamera = s.switchCamera;
        copy.X = s.X;
        copy.Y = s.Y;
        copy.yaw = s.yaw;
        copy.Z = s.Z;
    }

    //retrieve the flight control state entered at a given time
    public void pop(FlightCtrlState controls, double time)
    {
        if (times.Count == 0) return; //leave the transmitted controls intact

        double popTime = times.Peek();
        FlightCtrlState popState = states.Peek();
        while (times.Peek() < time)
        {
            popTime = times.Dequeue();
            popState = states.Dequeue();
        }

        copyFlightCtrlState(popState, controls);
    }

    void print(String s)
    {
        MonoBehaviour.print(s);
    }
}



