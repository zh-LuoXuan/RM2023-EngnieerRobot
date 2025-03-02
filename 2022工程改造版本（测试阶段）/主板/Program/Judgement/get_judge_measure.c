#include "get_judge_measure.h"

/*****************ϵͳ���ݶ���**********************/
extern ext_game_status_t       				GameState;					//0x0001
extern ext_game_result_t            		GameResult;					//0x0002
extern ext_game_robot_HP_t          		GameRobotSurvivors;			//0x0003
extern ext_event_data_t        				EventData;					//0x0101
extern ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
extern ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
extern ext_game_robot_status_t			  	GameRobotStat;				//0x0201
extern ext_power_heat_data_t		  		PowerHeatData;				//0x0202
extern ext_game_robot_pos_t					GameRobotPos;				//0x0203
extern ext_buff_t							BuffMusk;					//0x0204
extern aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
extern ext_robot_hurt_t						RobotHurt;					//0x0206
extern ext_shoot_data_t						ShootData;					//0x0207



//���ز���ϵͳ���ն˱�����ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const ext_game_status_t *get_game_state_t(void)//����״̬����
{
	return &GameState;
}

const ext_game_result_t *get_game_result_t(void)//�����������
{
	return &GameResult;
}

const ext_game_robot_HP_t *get_game_robot_survivors_t(void)//���������˴������
{
	return &GameRobotSurvivors;
}

const ext_event_data_t *get_event_data_t(void)//�����¼�����
{
	return &EventData;
}

const ext_supply_projectile_action_t *get_supply_projectile_action_t(void)//���ز���վ������ʶ����
{
	return &SupplyProjectileAction;
}

const ext_supply_projectile_booking_t *get_supply_projectile_booking_t(void)//���ز���վԤԼ�ӵ�����
{
	return &SupplyProjectileBooking;
}

const ext_game_robot_status_t *get_game_robot_state_t(void)//������״̬����
{
	return &GameRobotStat;
}

const ext_power_heat_data_t *get_power_heat_data_t(void)//ʵʱ������������
{
	return &PowerHeatData;
}

const ext_game_robot_pos_t *get_Robot_Pos_t(void)//������λ������
{
	return &GameRobotPos;
}

const ext_buff_t *get_buff_musk_t(void)//��������������
{
	return &BuffMusk;
}

const aerial_robot_energy_t *get_robot_energy_t(void)//���л���������״̬����
{
	return &AerialRobotEnergy;
}

const ext_robot_hurt_t *get_robot_hurt_t(void)//�˺�״̬����
{
	return &RobotHurt;
}

const ext_shoot_data_t *get_shoot_data_t(void)//ʵʱ�������
{
	return &ShootData;
}











