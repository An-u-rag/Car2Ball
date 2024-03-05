using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using System.Diagnostics;

public class CarAgent : Agent
{
    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;
    private int brakingInt;
    private bool isBraking;
    // private float currentBrakeForce;
    private float currentSteerAngle;
    private Transform wheelTransform;

    [SerializeField] private float motorforce;
    // [SerializeField] private float brakeForce;
    [SerializeField] private float maxSteeringAngle;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    public Transform Target;
    Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (this.transform.localPosition.y < 0)
        {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0.5f, 0);
            this.transform.rotation = Quaternion.identity;
            this.currentSteerAngle = 0;
            this.horizontalInput = 0f;
            this.verticalInput = 0f;
        }

        // Move the target to a new spot
        Target.localPosition = new Vector3(UnityEngine.Random.value * 40 - 20,
                                           0.5f,
                                           UnityEngine.Random.value * 40 - 20);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // isBraking = false;
        horizontalInput = actionBuffers.ContinuousActions[0];
        verticalInput = actionBuffers.ContinuousActions[1];
        // brakingInt = actionBuffers.DiscreteActions[0];
        // if (brakingInt == 1) { isBraking = true; }
        // update car
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        // floor is side-length of 1 so set reward to 1-dist to target
        float reward = 1 - distanceToTarget;
        if (reward < 0)
        {
            reward = 0;
        }
        AddReward(reward);

        // Fell off platform
        if (this.transform.localPosition.y < -1)
        {
            EndEpisode();
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        // Reached target
        if (collision.gameObject.name == "Target")
        {
            AddReward(1.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        // var discreteActionsOut = actionsOut.DiscreteActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
        // bool braking = Input.GetKey(KeyCode.Space);
        // if (braking) {
        //     discreteActionsOut[0] = 1;
        // }
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = verticalInput * motorforce;
        rearRightWheelCollider.motorTorque = verticalInput * motorforce;
        // currentBrakeForce = isBraking ? brakeForce : 0f;
        // if (isBraking)
        // {
        //     ApplyBraking();
        // }   
    }

    // private void ApplyBraking()
    // {
    //     frontLeftWheelCollider.brakeTorque = currentBrakeForce;
    //     frontRightWheelCollider.brakeTorque = currentBrakeForce;
    //     rearLeftWheelCollider.brakeTorque = currentBrakeForce;
    //     rearRightWheelCollider.brakeTorque = currentBrakeForce;
    // }

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
