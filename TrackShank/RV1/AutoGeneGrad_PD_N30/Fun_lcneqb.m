function FunOut=Fun_lcneqb(lcneqw_peri,delta_q_perimin,delta_qdot_perimin,delta_qddot_perimin,delta_a_perimin,delta_u_perimin,delta_q_perimax,delta_qdot_perimax,delta_qddot_perimax,delta_a_perimax,delta_u_perimax)
FunOut=[-delta_q_perimin(1)*lcneqw_peri(1);-delta_q_perimin(2)*lcneqw_peri(1);-delta_q_perimin(3)*lcneqw_peri(1);delta_q_perimax(1)*lcneqw_peri(1);delta_q_perimax(2)*lcneqw_peri(1);delta_q_perimax(3)*lcneqw_peri(1);-delta_q_perimin(4)*lcneqw_peri(1);-delta_q_perimin(5)*lcneqw_peri(1);-delta_q_perimin(6)*lcneqw_peri(1);delta_q_perimax(4)*lcneqw_peri(1);delta_q_perimax(5)*lcneqw_peri(1);delta_q_perimax(6)*lcneqw_peri(1);-delta_q_perimin(7)*lcneqw_peri(1);-delta_q_perimin(8)*lcneqw_peri(1);-delta_q_perimin(9)*lcneqw_peri(1);delta_q_perimax(7)*lcneqw_peri(1);delta_q_perimax(8)*lcneqw_peri(1);delta_q_perimax(9)*lcneqw_peri(1);-delta_qdot_perimin(1)*lcneqw_peri(2);-delta_qdot_perimin(2)*lcneqw_peri(2);-delta_qdot_perimin(3)*lcneqw_peri(2);delta_qdot_perimax(1)*lcneqw_peri(2);delta_qdot_perimax(2)*lcneqw_peri(2);delta_qdot_perimax(3)*lcneqw_peri(2);-delta_qdot_perimin(4)*lcneqw_peri(2);-delta_qdot_perimin(5)*lcneqw_peri(2);-delta_qdot_perimin(6)*lcneqw_peri(2);delta_qdot_perimax(4)*lcneqw_peri(2);delta_qdot_perimax(5)*lcneqw_peri(2);delta_qdot_perimax(6)*lcneqw_peri(2);-delta_qdot_perimin(7)*lcneqw_peri(2);-delta_qdot_perimin(8)*lcneqw_peri(2);-delta_qdot_perimin(9)*lcneqw_peri(2);delta_qdot_perimax(7)*lcneqw_peri(2);delta_qdot_perimax(8)*lcneqw_peri(2);delta_qdot_perimax(9)*lcneqw_peri(2);-delta_qddot_perimin(1)*lcneqw_peri(3);-delta_qddot_perimin(2)*lcneqw_peri(3);-delta_qddot_perimin(3)*lcneqw_peri(3);delta_qddot_perimax(1)*lcneqw_peri(3);delta_qddot_perimax(2)*lcneqw_peri(3);delta_qddot_perimax(3)*lcneqw_peri(3);-delta_qddot_perimin(4)*lcneqw_peri(3);-delta_qddot_perimin(5)*lcneqw_peri(3);-delta_qddot_perimin(6)*lcneqw_peri(3);delta_qddot_perimax(4)*lcneqw_peri(3);delta_qddot_perimax(5)*lcneqw_peri(3);delta_qddot_perimax(6)*lcneqw_peri(3);-delta_qddot_perimin(7)*lcneqw_peri(3);-delta_qddot_perimin(8)*lcneqw_peri(3);-delta_qddot_perimin(9)*lcneqw_peri(3);delta_qddot_perimax(7)*lcneqw_peri(3);delta_qddot_perimax(8)*lcneqw_peri(3);delta_qddot_perimax(9)*lcneqw_peri(3);-delta_a_perimin(1)*lcneqw_peri(4);-delta_a_perimin(2)*lcneqw_peri(4);-delta_a_perimin(3)*lcneqw_peri(4);delta_a_perimax(1)*lcneqw_peri(4);delta_a_perimax(2)*lcneqw_peri(4);delta_a_perimax(3)*lcneqw_peri(4);-delta_a_perimin(4)*lcneqw_peri(4);-delta_a_perimin(5)*lcneqw_peri(4);-delta_a_perimin(6)*lcneqw_peri(4);delta_a_perimax(4)*lcneqw_peri(4);delta_a_perimax(5)*lcneqw_peri(4);delta_a_perimax(6)*lcneqw_peri(4);-delta_u_perimin(1)*lcneqw_peri(5);-delta_u_perimin(2)*lcneqw_peri(5);-delta_u_perimin(3)*lcneqw_peri(5);delta_u_perimax(1)*lcneqw_peri(5);delta_u_perimax(2)*lcneqw_peri(5);delta_u_perimax(3)*lcneqw_peri(5);-delta_u_perimin(4)*lcneqw_peri(5);-delta_u_perimin(5)*lcneqw_peri(5);-delta_u_perimin(6)*lcneqw_peri(5);delta_u_perimax(4)*lcneqw_peri(5);delta_u_perimax(5)*lcneqw_peri(5);delta_u_perimax(6)*lcneqw_peri(5)];

end
