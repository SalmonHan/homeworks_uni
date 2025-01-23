using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;

public class tangent_bug : MonoBehaviour
{
    public float speed;
    private Rigidbody objRigidbody;
    public Vector3 direction;

    public float ray_length = 5f;
    public Transform goal;
    public Vector3 target;

    public GameObject target_object;

    public float local_minimum_count = 0.3f;
    public float current_local_minimum_count = 0f;
    private Vector3 old_position;


    public enum Mode
    {
        motion_to_goal, boundary_following, goto_stopover
    }

    public Mode mode;

    void Start()
    {
        mode = Mode.motion_to_goal;
        old_position = transform.position;
    }


    void Update()
    {
        current_local_minimum_count += Time.deltaTime;
        if(current_local_minimum_count > local_minimum_count)
        {
            //현재 위치가 old_position과 별 차이가 없다면 로컬 미니마에 빠진것으로 간주
            //print(Vector3.Distance(old_position, transform.position));
            if (Vector3.Distance(old_position, transform.position) < 0.5f)
            {
                mode = Mode.boundary_following;
            }
            current_local_minimum_count = 0f;

            old_position = transform.position;
        }



        target = get_nearest_point();

        if (mode == Mode.motion_to_goal)
        {

            direction = (target - gameObject.transform.position);
            direction.y = 0;
            direction = direction.normalized;
            transform.Translate(direction * speed * Time.deltaTime);
        }
        else if (mode == Mode.boundary_following)
        {
            //가장 가까운 레이를 기준으로 수직으로 이동

            float angleStep = 360f / 72;  // 각도 간격 계산
            target = new Vector3(10000, 0, 0);

            float min_distance = float.MaxValue;


            for (int i = 0; i < 72; i++)
            {
                float currentAngle = i * angleStep;
                Vector3 direction = new Vector3(Mathf.Cos(currentAngle * Mathf.Deg2Rad), 0, Mathf.Sin(currentAngle * Mathf.Deg2Rad));

                Ray ray = new Ray(transform.position, direction);
                RaycastHit hit;

                if (Physics.Raycast(ray, out hit, ray_length))
                {

                    if (min_distance > Vector3.Distance(transform.position, hit.point))
                    {
                        direction = hit.normal;
                    }

                }
                else
                {
                }

            }

            Quaternion rotation = Quaternion.Euler(0, 90, 0);
            direction = (rotation * direction).normalized;
            transform.Translate(direction * speed * Time.deltaTime);

        }

        target_object.transform.position = target;
    }




    

    private void OnTriggerEnter(Collider other)
    {

        if (other.gameObject.tag == "goal")
        {
            print("Success");
        }
    }




    private Vector3 get_nearest_point()
    {
        float angleStep = 360f / 72;  // 각도 간격 계산
        target = new Vector3(10000, 0, 0);

        float old_dist = ray_length;
        float new_dist = ray_length;

        float safeDistance = 0f;
         
        for (int i = 0; i < 72; i++)
        {
            float currentAngle = i * angleStep;
            Vector3 direction = new Vector3(Mathf.Cos(currentAngle * Mathf.Deg2Rad), 0, Mathf.Sin(currentAngle * Mathf.Deg2Rad));


            Ray ray = new Ray(transform.position, direction);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, ray_length))
            {
                Debug.DrawLine(transform.position, hit.point, Color.red);

                new_dist = Vector3.Distance(transform.position, hit.point);

                if (Mathf.Abs(new_dist - old_dist) > (ray_length * 0.5))
                {
                    if (Vector3.Distance(transform.position, target) + Vector3.Distance(target, goal.position) > new_dist + Vector3.Distance(hit.point, goal.position))
                    {
                        if (target != goal.position)
                            //target = hit.point;
                            target = hit.point + (hit.normal * safeDistance);
                    }
                }
            }
            else
            {
                Debug.DrawLine(transform.position, transform.position + direction * ray_length, Color.green);
                new_dist = float.MaxValue;
            }

            old_dist = new_dist;
        }

        //goal에 ray 발사, 아무것도 없다면 타겟은 goal
        direction = (goal.position - transform.position);
        direction.y = 0;
        direction = direction.normalized;

        Ray ray_to_goal = new Ray(transform.position, direction);
        RaycastHit hit_to_goal;

        if (Physics.Raycast(ray_to_goal, out hit_to_goal, ray_length))
        {
        }
        else
        {
            mode = Mode.motion_to_goal;
            target = goal.position;
        }


        return target;
    }


}




