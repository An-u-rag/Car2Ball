using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using System.Diagnostics;
using System.Security.Cryptography;

public class RollerAgent : Agent
{
    // Start is called before the first frame update
    Rigidbody rBody;
    //GameObject Wheel;
    void Start()
    {
        //Wheel = transform.Find("Wheel").gameObject;
        rBody = GetComponent<Rigidbody>();
    }

    public Transform Target;
    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (this.transform.localPosition.y < 0)
        {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0.25f, 4);
        }

        // Move the target to a new spot
        Target.localPosition = new Vector3(UnityEngine.Random.value * 8 - 4,
                                           0.5f,
                                           UnityEngine.Random.value * 8 - 4);
    }

    public float forceMultiplier = 10;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions, size = 2
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];
        rBody.AddForce(controlSignal * forceMultiplier);

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        // floor is side-length of 1 so set reward to 1-dist to target
        float reward=0;
        UnityEngine.Debug.Log("Distance to Target: " + distanceToTarget);

        if (distanceToTarget >= 1.01f)
        {
            reward = 1.0f / (float)(Math.Pow(distanceToTarget - 1, 2.0f));
        }

        if (reward < 0)
        {
            reward = 0;
        }
        AddReward(reward);
        UnityEngine.Debug.Log("Reward: " + reward);

        // Reached target
        if (distanceToTarget <= 1.01f)
        {
            EndEpisode();
        }

        // Fell off platform
        else if (this.transform.localPosition.y < 0)
        {
            EndEpisode();
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }

}
