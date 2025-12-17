export interface Attitude {
    roll_deg: number;
    pitch_deg: number;
    yaw_deg: number;
}

export const DefaultAttitude : Attitude = 
{
    roll_deg : 0,
    pitch_deg : 0,
    yaw_deg : 0,
};