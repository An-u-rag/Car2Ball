using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using System.Diagnostics;

public class CarGoalAgent : Agent
{
    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;
    private int brakingInt;
    private bool isBraking;
    private float currentBrakeForce;
    private float currentSteerAngle;
    private Transform wheelTransform;

    [SerializeField] private float motorforce;
    [SerializeField] private float brakeForce;
    [SerializeField] private float maxSteeringAngle;
    [SerializeField] private int eps_range;
    [SerializeField] private float eps_scale;
    [SerializeField] private int sensor_range;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [SerializeField] private Transform VectorSensorPoint;

    [SerializeField] private Transform Arena;
    [SerializeField] private Transform Target;
    [SerializeField] private Transform Goal;

    Rigidbody rBody;
    Vector3 arenaCenter;
    Vector3[] goal_positions;
    Quaternion[] goal_rotations;
    bool collided_with_ball;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
        arenaCenter = Arena.transform.position;
        goal_positions = new [] {arenaCenter + new Vector3(24f, -1.501f, 0f), 
                                arenaCenter + new Vector3(-24f, -1.501f, 0f), 
                                arenaCenter + new Vector3(0f, -1.501f, 24f), 
                                arenaCenter + new Vector3(0f, -1.501f, -24f)};
        goal_rotations = new [] {Quaternion.Euler(0, 0, 0), 
                                 Quaternion.Euler(0, 0, 0),
                                 Quaternion.Euler(0, 90, 0), 
                                 Quaternion.Euler(0, 90, 0)};
    }

    public override void OnEpisodeBegin()
    {
        // Zero out car momentum
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;
        this.transform.localPosition = new Vector3(0, 0.5f, 0);
        this.transform.rotation = Quaternion.identity;
        this.currentSteerAngle = 0;
        this.horizontalInput = 0f;
        this.verticalInput = 0f;

        // initialize boolean to false
        collided_with_ball = false;

        // zero out ball velocity
        Rigidbody tarbody = Target.GetComponent<Rigidbody>();
        tarbody.velocity = Vector3.zero;
        tarbody.angularVelocity = Vector3.zero;

        // Move the target to a new spot
        Target.localPosition = new Vector3(UnityEngine.Random.value * 40 - 20,
                                           0.5f,
                                           UnityEngine.Random.value * 40 - 20);
        int goalidx = UnityEngine.Random.Range(0, 4);
        Goal.position = goal_positions[goalidx];
        Goal.rotation = goal_rotations[goalidx];
    }

    public override void CollectObservations(VectorSensor sensor) {
        RaycastHit hit;
        float dist;
        Ray fwd_ray = new Ray(VectorSensorPoint.transform.position, VectorSensorPoint.forward);
        // add noise to the distance value and scale it by the 
        int eps = UnityEngine.Random.Range(-1*eps_range, 1*eps_range);
        if (Physics.Raycast(fwd_ray, out hit, sensor_range)) 
        {
            // get distance from raycast hit
            dist = hit.distance;
            // add noise to distance
            dist = dist + (eps*eps_scale*dist);
        } else {
            dist = sensor_range + eps;
        }

        // Pass current car state to model
        sensor.AddObservation(horizontalInput);
        sensor.AddObservation(verticalInput);
        // Pass distance reading to model
        sensor.AddObservation(dist);
        // Visual inputs are added automatically by camerasensor component
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        isBraking = false;
        horizontalInput = actionBuffers.ContinuousActions[0];
        verticalInput = actionBuffers.ContinuousActions[1];
        // RC cars do not have brakes. The wheels stop when no forward or backward input is given
        isBraking = (verticalInput == 0) ? true :isBraking = false;

        // update car
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        // Rewards
        if (collided_with_ball) {
            float distanceToTarget = Vector3.Distance(this.transform.position, Target.position);

            // floor is side-length of 1 so set reward to 1-dist to target
            double reward =  1/Math.Pow(distanceToTarget, 2);
            float r = (float) reward;
            AddReward(r);
        } else {
            float distanceBallToGoal = Vector3.Distance(Target.transform.position, Goal.transform.position);
            double reward = 1/Math.Pow(distanceBallToGoal, 2);
            float r = (float) reward;
            AddReward(r);
        }

        // Fell off platform
        if (this.transform.localPosition.y < -1 || this.transform.localPosition.y > 1)
        {
            EndEpisode();
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        // Reached target
        if ((collision.gameObject.name == "Target") && !(collided_with_ball))
        {
            AddReward(1.0f);
            collided_with_ball = true;
        }
    }

    public void BallInGoal()
    {
        AddReward(10f);
        EndEpisode();
    }

    public void BallTouchedWall()
    {
        // the car can't really get a ball away from the wall (especially corners) so end the episode
        EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActionsOut = actionsOut.DiscreteActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = verticalInput * motorforce;
        rearRightWheelCollider.motorTorque = verticalInput * motorforce;
        currentBrakeForce = isBraking ? brakeForce : 0f;
        if (isBraking)
        {
            ApplyBraking();
        } else {
            RemoveBreaking();
        }
    }

    private void ApplyBraking()
    {
        frontLeftWheelCollider.brakeTorque = currentBrakeForce;
        frontRightWheelCollider.brakeTorque = currentBrakeForce;
        rearLeftWheelCollider.brakeTorque = currentBrakeForce;
        rearRightWheelCollider.brakeTorque = currentBrakeForce;
    }

    private void RemoveBreaking()
    {
        frontLeftWheelCollider.brakeTorque = 0;
        frontRightWheelCollider.brakeTorque = 0;
        rearLeftWheelCollider.brakeTorque = 0;
        rearRightWheelCollider.brakeTorque = 0;
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteeringAngle*horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform transform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        transform.rotation = rot;
        transform.position = pos;
    }   
}
