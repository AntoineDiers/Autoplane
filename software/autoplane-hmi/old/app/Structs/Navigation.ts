import { Attitude } from './Attitude';
import { Position } from './Position';

export interface Navigation {
    attitude: Attitude;
    position: Position
    speed_m_s: number;
}