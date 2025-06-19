#include <iostream>
#include <vector>
#include "gurobi_c++.h"
#include "milp_solver_interface.h"
#include "fpga.h"
#include "vcu128_fine_grained.h"

using namespace std;

typedef vector<GRBVar>          GRBVarArray;
typedef vector<GRBVarArray>     GRBVar2DArray;
typedef vector<GRBVar2DArray>   GRBVar3DArray;
typedef vector<GRBVar3DArray>   GRBVar4DArray;

static unsigned long H, W;
static unsigned long num_slots;
static unsigned long num_rows;
static unsigned long num_forbidden_slots;
static unsigned long BIG_M = 100000000;
static unsigned long num_clk_regs;

static unsigned long wasted_clb_vcu128, wasted_bram_vcu128, wasted_dsp_vcu128;

static int status;
static unsigned long delta_size;
static unsigned long clb_max  = 300000;
static unsigned long bram_max = 100000;
static unsigned long dsp_max  = 100000;
static unsigned long clb_per_tile;
static unsigned long bram_per_tile;
static unsigned long dsp_per_tile;

static vector<unsigned long> clb_req_vcu128(MAX_SLOTS);
static vector<unsigned long> bram_req_vcu128(MAX_SLOTS);
static vector<unsigned long> dsp_req_vcu128(MAX_SLOTS);

static vcu128_fine_grained vcu128_coordinates;

const unsigned long num_fbdn_edge = 32+14; // DSP+BRAM

int solve_milp_vcu128(param_from_solver *to_sim)
{
    unsigned long status, i, j, k, l, m;
    unsigned long dist_0, dist_1, dist_2;

    //define variables
    try {
        GRBEnv env = GRBEnv();
        GRBConstr* c = NULL;
        GRBModel model = GRBModel(env);

            delta_size = num_slots;
            //delta_size = num_forbidden_slots;


        //Variable definition

        /**********************************************************************
         name: x
         type: integer
         func: x[i][k] represent the left and right x coordinate of slot 'i'
        ***********************************************************************/

        GRBVar2DArray x(num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(2);
            x[i] = each_slot;

            for(k = 0; k < 2; k++)
                x[i][k] = model.addVar(0.0, W, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
         name: y
         type: integer
         func: y[i] represents the bottom left y coordinate of slot 'i'
        ***********************************************************************/

        GRBVarArray y (num_slots);
        for(i = 0; i < num_slots; i++) {
            y[i] = model.addVar(0.0, num_clk_regs, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
         name: w
         type: integer
         func: w[i] represents the width of the slot 'i'
        ***********************************************************************/

        GRBVarArray w (num_slots);
        for(i = 0; i < num_slots; i++) {
            w[i] = model.addVar(0.0, W, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
         name: h
         type: integer
         func: h[i] represents the height of slot 'i'
        ***********************************************************************/

        GRBVarArray h (num_slots);
        for(i = 0; i < num_slots; i++) {
            h[i] = model.addVar(0.0, H, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
         name: z
         type: binary
         func: z[i][k][][] is used to define the constraints on the distribution of
               resource on the FPGA fabric

               z[0][i][x_1/2][] == for clb
               z[1][i][x_1/2][] == for bram
               z[2][i][x_1/2][] == for dsp
        ***********************************************************************/

        GRBVar4DArray z(6);
        for(i = 0; i < 6; i++) {
            GRBVar3DArray each_slot(num_slots);
            z[i] = each_slot;

            for(k = 0; k < num_slots; k++)  {
                GRBVar2DArray x_coord(2);
                z[i][k] = x_coord;

                for(j = 0; j < 2; j++) {
                    GRBVarArray constrs(100);
                    z[i][k][j] = constrs;

                    for(l = 0; l < 100; l++)
                        z[i][k][j][l] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                }
            }
        }

       /**********************************************************************
         name: clb
         type: integer
         func: clb[i][k] represents the number of clbs in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total numbe clb in slot 'i' is then calculated by
                clb in 'i' = clb[i][1] - clb[i][0]
        ***********************************************************************/

        GRBVar2DArray clb (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(2);
            clb[i] = each_slot;

            for(k = 0; k < 2; k++)
                clb[i][k] = model.addVar(0.0, clb_max, 0.0, GRB_INTEGER);
        }

       /**********************************************************************
         name: bram
         type: integer
         func: bram[i][k] represents the number of brams in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total numbe brams in slot 'i' is then calculated by
                bram in 'i' = bram[i][1] - bram[i][0]
        ***********************************************************************/

        GRBVar2DArray bram (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(2);
            bram[i] = each_slot;

            for(k = 0; k < 2; k++)
                bram[i][k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
         name: dsp
         type: integer
         func: dsp[i][k] represents the number of dsp in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total numbe brams in slot 'i' is then calculated by
                dsp in 'i' = dsp[i][1] - dsp[i][0]
        ***********************************************************************/

        GRBVar2DArray dsp (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(2);
            dsp[i] = each_slot;

            for(k = 0; k < 2; k++)
                dsp[i][k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
        }

       /**********************************************************************
         name: clb-fbdn
         type: integer
         func: clb_fbdn[i][k] represents the number of clbs in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total number of clb in forbidden region 'i' is then calculated by
                clb_fbdn in 'i' = clb_fbdn[i][1] - clb_fbdn[i][0]
        ***********************************************************************/
        GRBVar3DArray clb_fbdn (2); //TODO: This should be modified to num_forbidden_slots
        for(i= 0; i < 2; i++) {
            GRBVar2DArray each_slot(num_slots);
            clb_fbdn[i] = each_slot;

            for(j = 0; j < num_slots; j++) {
                GRBVarArray each_slot_fbdn(2);

                clb_fbdn[i][j] = each_slot_fbdn;
                for(k = 0; k < 2; k++)
                    clb_fbdn[i][j][k] = model.addVar(0.0, clb_max, 0.0, GRB_INTEGER);
            }
        }

        /**********************************************************************
        The following 3 variables record the total number of CLB, BRAM and DSP
        in forbidden regions.
        ***********************************************************************/

        GRBVarArray clb_fbdn_tot (num_slots);
        for(i = 0; i < num_slots; i++) {
            clb_fbdn_tot[i] = model.addVar(0, clb_max, 0.0, GRB_INTEGER);
        }

        GRBVarArray bram_fbdn_tot (num_slots);
        for(i = 0; i < num_slots; i++) {
            bram_fbdn_tot[i] = model.addVar(0, bram_max, 0.0, GRB_INTEGER);
        }

        GRBVarArray dsp_fbdn_tot (num_slots);
        for(i = 0; i < num_slots; i++) {
            dsp_fbdn_tot[i] = model.addVar(0, dsp_max, 0.0, GRB_INTEGER);
        }


       /**********************************************************************
         name: bram_fbdn
         type: integer
         func: bram[i][k] represents the number of brams in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total number of brams in the forbidden region 'i' is then calculated by
                bram in 'i' = bram_fbdn[i][1] - bram_fbdn[i][0]
        ***********************************************************************/
        GRBVar3DArray bram_fbdn (2);
        for(j = 0; j < 2; j++) {
            GRBVar2DArray each_slot (num_slots);
            bram_fbdn[j] = each_slot;

            for(i = 0; i < num_slots; i++) {
                GRBVarArray each_slot_fbdn(2);
                bram_fbdn[j][i] = each_slot_fbdn;

                for(k = 0; k < 2; k++)
                    bram_fbdn[j][i][k] = model.addVar(0.0, bram_max, 0.0, GRB_INTEGER);
            }
        }

        /**********************************************************************
         name: dsp_fbdn
         type: integer
         func: dsp_fbdn[i][k] represents the number of dsp in (0, x_1) & (0, x_2)
               in a single row.
               'k' = 0 -> x_1
               'k' = 1 -> x_2

               the total numbe dsps in forbidden region 'i' is then calculated by
                dsp in 'i' = dsp_fbdn[i][1] - dsp_fbdn[i][0]
        ***********************************************************************/
        GRBVar3DArray dsp_fbdn (2);
        for(j = 0; j < 2; j++) {
            GRBVar2DArray each_fbdn_slot (num_slots);
            dsp_fbdn[j] = each_fbdn_slot;

            for(i = 0; i < num_slots; i++) {
                GRBVarArray each_slot(2);
                dsp_fbdn[j][i] = each_slot;

                for(k = 0; k < 2; k++)
                    dsp_fbdn[j][i][k] = model.addVar(0.0, dsp_max, 0.0, GRB_INTEGER);
            }
        }

        /**********************************************************************
         name: beta
         type: binary
         func: beta[i][k] = 1 if clock region k is part of slot 'i'
        ***********************************************************************/

        GRBVar2DArray beta (num_slots);
        for(i = 0; i < num_slots; i++) {
                GRBVarArray for_each_clk_reg(num_clk_regs);
                beta[i] = for_each_clk_reg;

                for(j = 0; j < num_clk_regs; j++) {
                     beta[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
            }
        }

        /**********************************************************************
         name: tau
         type: integer
         func: tau[i][k] is used to linearize the function which is used to compute
               the number of available resources. The first index is used to
               denote the type of resource and the second is used to denote
               the slot
        ***********************************************************************/
        GRBVar3DArray tau (3); //for clb, bram, dsp
        for(i = 0; i < 3; i++) {
            GRBVar2DArray each_slot(num_slots);
            tau[i] = each_slot;

            for(l = 0; l < num_slots; l++) {
                GRBVarArray for_each_clk_reg(num_clk_regs);

                tau[i][l] = for_each_clk_reg;
                for(k = 0; k < num_clk_regs; k++) {
                      tau[i][l][k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
                }
            }
        }

        /**********************************************************************
         name: tau_fbdn
         type: integer
         func: tau_fbdn[i][k] is used to linearize the function which is used to compute
               the number of available resources. The first index is used to
               denote the type of resource and the second is used to denote
               the slot
        ***********************************************************************/
        GRBVar4DArray tau_fbdn (2); //for forbidden clb, bram, dsp
        for(j = 0; j < 2; j++) {
            GRBVar3DArray each_slot_fbdn(3);
            tau_fbdn[j] = each_slot_fbdn;

            for(i=0; i < 3; i++) {
                GRBVar2DArray each_slot(num_slots);
                tau_fbdn[j][i] = each_slot;

                for(l = 0; l < num_slots; l++) {
                    GRBVarArray for_each_clk_reg(num_clk_regs);

                    tau_fbdn[j][i][l] = for_each_clk_reg;
                    for(k = 0; k < num_clk_regs; k++) {
                        tau_fbdn[j][i][l][k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
                    }
                }
            }
        }


       /**********************************************************************
         name: gamma
         type: binary
         func: gamma[i][k] = 1 iff bottom left x coordinate of slot 'i' is found
               to the left of the bottom left x coordinate of slot 'k'

               gamma[i][k] = 1 if x_i <= x_k
        ***********************************************************************/

        GRBVar2DArray gamma (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);
            gamma[i] = each_slot;

            for(k = 0; k < num_slots; k++)
                gamma[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        /**********************************************************************
         name: theta
         type: binary
         func: theta[i][k] = 1 iff the bottom left y coordinate of slot 'i' is
               found below the bottom left y coordinate of slot 'k'

               theta[i][k] = 1 if y_i <= y_k
        ***********************************************************************/

        GRBVar2DArray theta (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);
            theta[i] = each_slot;

            for(k = 0; k < num_slots; k++)
                theta[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }
        /**********************************************************************
         name: Gamma
         type: binary
         func: Gamma[i][k] = 1 iff x_i + w_i >= x_k
        ***********************************************************************/

        GRBVar2DArray Gamma (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);

            Gamma[i] = each_slot;
            for(k = 0; k < num_slots; k++)
                Gamma[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        /**********************************************************************
         name: Alpha
         type: binary
         func: Alpha[i][k] = 1 iff x_k + w_k >= x_i
        ***********************************************************************/

        GRBVar2DArray Alpha (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);
            Alpha[i] = each_slot;

            for(k = 0; k < num_slots; k++)
                Alpha[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        /**********************************************************************
         name: Omega
         type: binary
         func: Omega[i][k] = 1 iff y_i + h_i >= y_k
        ***********************************************************************/

        GRBVar2DArray Omega(num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);
            Omega[i] = each_slot;

           for(k = 0; k < num_slots; k++)
                Omega[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        /**********************************************************************
         name: Psi
         type: binary
         func: Psi[i][k] = 1 iff  y_k + h_k >= y_i
        ***********************************************************************/

        GRBVar2DArray Psi(num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(num_slots);
            Psi[i] = each_slot;

            for(k = 0; k < num_slots; k++)
                Psi[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        /**********************************************************************
         name: delta
         type: binary
         func: delta[i][k] = 1 if slot 'i' and 'k' share at least one tile
        ***********************************************************************/
        GRBVar3DArray delta (2);
        for(j = 0; j < 2; j++) {
            GRBVar2DArray each_slot(delta_size);

            delta[j] = each_slot;
            for(i = 0; i < delta_size; i++) {
                GRBVarArray each_slot_1(delta_size);

                delta[j][i] = each_slot_1;
                for(k = 0; k < delta_size; k++)
                    delta[j][i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
            }
        }
//#ifdef fbdn
        /**********************************************************************
         name: kappa
         type: binary
         func: this variable is used to formulate the constraint on wasted resources
                kappa[i][k] is a variable to constrain wasted resource type i in slot k
        ***********************************************************************/
        GRBVar3DArray kappa(num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVar2DArray each_slot(num_fbdn_edge);

            kappa[i] = each_slot;
            for(k = 0; k < num_fbdn_edge; k++){
                GRBVarArray each_slot_1 (2);
                  kappa[i][k] = each_slot_1;

                for(j = 0; j < 2; j++)
                  kappa[i][k][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
            }
        }

        /**********************************************************************
         name: wasted
         type: integer
         func: centroid[i][k] represents the wasted resources in each slot
               k = 0 => clb
               k = 1 => bram
               k = 2 => dsp
        ***********************************************************************/

        GRBVar2DArray wasted (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(3);
            wasted[i] = each_slot;

            for(k = 0; k < 3; k++)
                wasted[i][k] = model.addVar(0.0,  GRB_INFINITY, 0.0, GRB_INTEGER);
        }

        /**********************************************************************
          name: centroid
          type: integer
          func: centroid[i][k] represents the centroid of a slot i.
                centroid[i]0] is the centroid on the x axis while
                centroid [i][1] is the centroid on the y axis
        ***********************************************************************/

        GRBVar2DArray centroid (num_slots);
        for(i = 0; i < num_slots; i++) {
            GRBVarArray each_slot(2);
            centroid[i] = each_slot;

            for(k = 0; k < 2; k++)
                centroid[i][k] = model.addVar(0.0,  GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        }

        /**********************************************************************
          name: dist
          type: integer
          func: this variable represents the distance between the centroids of
                two regions. dist[i][k][0] represents the distance on the x axis
                between slots i and slot k while dist[i][k][1] represents the
                distance on the y axis between slots i and k.
         ***********************************************************************/
        GRBVar3DArray dist (num_slots);
        for(j = 0; j < num_slots; j++) {
            GRBVar2DArray each_slot(num_slots);

            dist[j] = each_slot;
            for(i = 0; i < num_slots; i++) {
                GRBVarArray each_slot_1(2);

                dist[j][i] = each_slot_1;
                for(k = 0; k < 2; k++)
                    dist[j][i][k] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            }
        }

        model.update();

        /********************************************************************
        Constr 1.1: The x coordinates must be constrained not to exceed
                      the boundaries of the fabric
        ********************************************************************/
        for(i = 0; i < num_slots; i++) {
            //for(k = 0; k< num_slots; k++) {
                model.addConstr(w[i] == x[i][1] - x[i][0], "1");
                //model.addConstr(x[i][0] <= W-1, "2");
                //model.addConstr(x[i][1] <= W, "3");
                model.addConstr(x[i][1] - x[i][0] >= 1, "4");
            //}
        }

        /********************************************************************
        Constr 1.2: The binary variables representing the rows must be
                    contigious i.e, if a region occupies clock region 1 and 3
                    then it must also occupy region 2
        ********************************************************************/
        for(i = 0; i < num_slots; i++) {
             for(j = 0; j < num_clk_regs - 2; j++) {
                 if(num_clk_regs > 2)
                     model.addConstr(beta[i][j+1] >= beta[i][j] + beta[i][j+2] - 1, "5");
             }
         }

        /************************************************************************
        Constr 1.3: The height of slot 'i' must be the sum of all clbs in the slot
        *************************************************************************/
        for(i = 0; i < num_slots; i++) {
            GRBLinExpr exp;
            for(j = 0; j < num_clk_regs; j++)
                exp += beta[i][j];
            model.addConstr(h[i] == exp, "6");
        }

        /******************************************************************
        Constr 1.4: y_i must be constrained not to be greater than the
                    lowest row
        ******************************************************************/
        for(i = 0; i < num_slots; i++) {
            GRBLinExpr exp_y;
            for(j = 0; j < num_clk_regs; j++)
                model.addConstr(y[i] <= (H - beta[i][j] * (H - j)), "99");
            model.addConstr(y[i] + h[i] <= H, "100");
        }

        int flora_x;
        int bram_x, dsp_x;
        int clb_offset;
        int constr_i;
        for(i = 0; i < num_slots; i++) {
            for(k = 0; k < 2; k++) {
                std::cout << "i=" << i << ", k=" << k << std::endl;

                GRBLinExpr exp[3];
                int z_l[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

                flora_x = 0;
                dsp_x = 0;
                bram_x = 0;
                clb_offset = 0;
                constr_i = 1;

                // first segment of piecewise function (no offset)
                model.addConstr(clb[i][k] >= x[i][k] - BIG_M * (1 - z[0][i][k][z_l[0][1]++]), std::to_string(constr_i++));
                model.addConstr(x[i][k] >= clb[i][k] - BIG_M * (1 - z[0][i][k][z_l[0][2]++]), std::to_string(constr_i++));
                model.addConstr(bram[i][k] >= bram_x  - BIG_M * (1 - z[1][i][k][z_l[1][1]++]), std::to_string(constr_i++));
                model.addConstr(bram_x >= (bram[i][k])  - BIG_M * (1 - z[1][i][k][z_l[1][2]++]), std::to_string(constr_i++));
                model.addConstr(dsp[i][k] >= (dsp_x - BIG_M * (1 - z[2][i][k][z_l[2][1]++])), std::to_string(constr_i++));
                model.addConstr(dsp_x >= dsp[i][k] - BIG_M * (1 - z[2][i][k][z_l[2][2]++]), std::to_string(constr_i++));

                // scan each x-coordinate for its type
                for (flora_x = 0; flora_x < VCU128_WIDTH; ++flora_x) {
#ifdef FLORA_VERBOSE
                    std::cout << "Resource @ x=" << flora_x << ", i=" << i << ", k=" << k << " is " << vcu128_coordinates.fg[flora_x].type_of_res << " " << z_l[2][0] << std::endl;
#endif
                    if (vcu128_coordinates.fg[flora_x].type_of_res == BRAM) {
                        /******************************************************************
                        Constr 2.1: The same thing as constr 1.0.0 is done for the bram
                                      which has the following piecewise distribution on
                                      the fpga fabric
                        ******************************************************************/
                        model.addConstr(BIG_M * z[1][i][k][z_l[1][0]++]  >= (flora_x + 1) - x[i][k], std::to_string(constr_i++));
                        model.addConstr(BIG_M * z[1][i][k][z_l[1][0]++]  >= x[i][k] - (flora_x), std::to_string(constr_i++));
#ifdef FLORA_VERBOSE
                        std::cout << "RAM: " << (flora_x + 1) << " - x[i][k]; x - " << (flora_x) << std::endl;
#endif

                        ++bram_x;
                        model.addConstr(bram[i][k] >= bram_x  - BIG_M * (1 - z[1][i][k][z_l[1][1]++]) -
                                                           BIG_M * (1 - z[1][i][k][z_l[1][1]++]), std::to_string(constr_i++));
                        model.addConstr(bram_x >= (bram[i][k])  - BIG_M * (1 - z[1][i][k][z_l[1][2]++]) -
                                                       BIG_M * (1 - z[1][i][k][z_l[1][2]++]), std::to_string(constr_i++));

                    }
                    else if (vcu128_coordinates.fg[flora_x].type_of_res == DSP) {
                        /******************************************************************
                        Constr 2.2: The same thing as constr 1.0.0 is done for the dsp
                                      which has the following piecewise distribution on
                                      the fpga fabric
                        ******************************************************************/

                        model.addConstr(BIG_M * z[2][i][k][z_l[2][0]++]  >= (flora_x + 1) - x[i][k], std::to_string(constr_i++));
                        model.addConstr(BIG_M * z[2][i][k][z_l[2][0]++]  >= x[i][k] - (flora_x), std::to_string(constr_i++));
#ifdef FLORA_VERBOSE
                        std::cout << "DSP: " << (flora_x + 1) << " - x[i][k]; x - " << (flora_x) << std::endl;
#endif

                        ++dsp_x;
                        model.addConstr(dsp[i][k] >= (dsp_x - BIG_M * (1 - z[2][i][k][z_l[2][1]++]) -
                                                BIG_M * (1 - z[2][i][k][z_l[2][1]++])), std::to_string(constr_i++));
                        model.addConstr(dsp_x >= (dsp[i][k])  - BIG_M * (1 - z[2][i][k][z_l[2][2]++]) -
                                                BIG_M * (1 - z[2][i][k][z_l[2][2]++]), std::to_string(constr_i++));

                    }
                    else {
                        continue;
                    }

                    /******************************************************************
                    Constr 2.0: The clb fingerprint on the FPGA is described using the following
                                piecewise function.
                                The piecewise function is then transformed into a set
                                of MILP constraints using the intermediate variable z
                    ******************************************************************/
                    model.addConstr(BIG_M * z[0][i][k][z_l[0][0]++] >= (flora_x + 1) - x[i][k], std::to_string(constr_i++));
                    model.addConstr(BIG_M * z[0][i][k][z_l[0][0]++] >= x[i][k] - (flora_x), std::to_string(constr_i++));
#ifdef FLORA_VERBOSE
                    std::cout << "CLB: " << (flora_x + 1) << " - x[i][k]; x - " << (flora_x) << "; x[i][k] - " << clb_offset << std::endl;
#endif

                    clb_offset++;
                    model.addConstr(clb[i][k] >= (x[i][k] - clb_offset)  - BIG_M * (1 - z[0][i][k][z_l[0][1]++]) -
                                                        BIG_M * (1 - z[0][i][k][z_l[0][1]++]), std::to_string(constr_i++));
                    model.addConstr(x[i][k] - clb_offset >= (clb[i][k])  - BIG_M * (1 - z[0][i][k][z_l[0][2]++]) -
                                                        BIG_M * (1 - z[0][i][k][z_l[0][2]++]), std::to_string(constr_i++));

                }

                // end of row constraints
                model.addConstr(BIG_M * z[0][i][k][z_l[0][0]++] >= W + 1 - x[i][k], std::to_string(constr_i++));
                model.addConstr(BIG_M * z[1][i][k][z_l[1][0]++] >= W + 1 - x[i][k], std::to_string(constr_i++));
                model.addConstr(BIG_M * z[2][i][k][z_l[2][0]++] >= W + 1 - x[i][k], std::to_string(constr_i++));

                for (int n = 0; n < 3; n++) {
                    for(m = 0; m < z_l[n][0]; m++) {
                        exp[n] += z[n][i][k][m];
                    }
                    model.addConstr(exp[n] <= (z_l[n][0] + 1) /2);
                }
            }

            /*************************************************************************
            Constraint 4.1: partition edges cannot border BRAM or DSP slices
            **************************************************************************/
            j = 0;
            // scan each x-coordinate for its type
            for (flora_x = 0; flora_x < VCU128_WIDTH; ++flora_x) {
                if (vcu128_coordinates.fg[flora_x].type_of_res == BRAM ||
                    vcu128_coordinates.fg[flora_x].type_of_res == DSP) {
                    l = 0;

#ifdef FLORA_VERBOSE
                    std::cout << "x[i][0] - " << (flora_x) << "; x[i][1] - " << (flora_x + 1) << std::endl;
#endif

                    // left boundary
                    model.addConstr(x[i][0] - (flora_x) <= -0.01 + kappa[i][j][l] * BIG_M, "edge_con");
                    model.addConstr(x[i][0] - (flora_x) >= 0.01 - (1 - kappa[i][j][l]) * BIG_M, "edge_con_1");
                    ++l;

                    // right boundary
                    model.addConstr(x[i][1] - (flora_x + 1) <= -0.01 + kappa[i][j][l] * BIG_M, "edge_con_2");
                    model.addConstr(x[i][1] - (flora_x + 1) >= 0.01 - (1 - kappa[i][j][l]) * BIG_M, "edge_con_3");

                    // increment number of forbidden boundaries
                    ++j;
                }
            }
        }


        //constr for res
        /*********************************************************************
          Constr 2.3: There must be enough clb, bram and dsp inside the slot
        **********************************************************************/
        for(i = 0; i < num_slots; i++) {
            GRBLinExpr exp_tau, exp_clb, exp_bram, exp_dsp;
            GRBLinExpr exp_clb_fbdn, exp_bram_fbdn, exp_dsp_fbdn;
            for(j = 0; j < num_clk_regs; j++) {
                //CLB constraints
                model.addConstr(tau[0][i][j] <= 10000 * beta[i][j], "58");
                model.addConstr(tau[0][i][j] <= clb[i][1] - clb[i][0], "59");
                model.addConstr(tau[0][i][j] >= (clb[i][1] - clb[i][0]) - (1 - beta[i][j]) * clb_max, "60");
                model.addConstr(tau[0][i][j] >= 0, "15");

/*                //CLB_FBDN 0
                model.addConstr(tau_fbdn[0][0][i][j] <= 100000 * (beta_fbdn[j] + beta[i][j] - 1), "58");
                model.addConstr(tau_fbdn[0][0][i][j] <= clb_fbdn[0][i][1] - clb_fbdn[0][i][0], "59");
                model.addConstr(tau_fbdn[0][0][i][j] >= (clb_fbdn[0][i][1] - clb_fbdn[0][i][0]) - (2 - beta[i][j] - beta_fbdn[j]) * clb_max, "60");
                model.addConstr(tau_fbdn[0][0][i][j] >= 0, "15");

                //CLB_FBDN 1
                model.addConstr(tau_fbdn[1][0][i][j] <= 10000 * (beta_fbdn[j] + beta[i][j] - 1),  "58");
                model.addConstr(tau_fbdn[1][0][i][j] <= clb_fbdn[1][i][1] - clb_fbdn[1][i][0], "59");
                model.addConstr(tau_fbdn[1][0][i][j] >= (clb_fbdn[1][i][1] - clb_fbdn[1][i][0]) - (2 - beta[i][j] - beta_fbdn[j]) * clb_max, "60");
                model.addConstr(tau_fbdn[1][0][i][j] >= 0, "15");
*/
                //BRAM constraints
                model.addConstr(tau[1][i][j] <= 1000 * beta[i][j], "61");
                model.addConstr(tau[1][i][j] <= bram[i][1] - bram[i][0], "62");
                model.addConstr(tau[1][i][j] >= (bram[i][1] - bram[i][0]) - (1 - beta[i][j]) * bram_max, "63");
                model.addConstr(tau[1][i][j] >= 0, "53");
/*
                //BRAM_fbdn constraints
                model.addConstr(tau_fbdn[0][1][i][j] <= 10000 * (beta_fbdn[j] + beta[i][j] - 1), "61");
                model.addConstr(tau_fbdn[0][1][i][j] <= bram_fbdn[0][i][1] - bram_fbdn[0][i][0], "62");
                model.addConstr(tau_fbdn[0][1][i][j] >= (bram_fbdn[0][i][1] - bram_fbdn[0][i][0]) - (2 - beta[i][j] - beta_fbdn[j]) * bram_max, "63");
                model.addConstr(tau_fbdn[0][1][i][j] >= 0, "53");
*/
                //DSP constraints
                model.addConstr(tau[2][i][j] <= 1000 * beta[i][j], "64");
                model.addConstr(tau[2][i][j] <= dsp[i][1] - dsp[i][0], "65");
                model.addConstr(tau[2][i][j] >= (dsp[i][1] - dsp[i][0]) - (1 - beta[i][j]) * dsp_max, "66");
                model.addConstr(tau[2][i][j] >= 0, "67");
/*
                //DSP_fbdn constraints
                model.addConstr(tau_fbdn[0][2][i][j] <= 10000 * (beta_fbdn[j] + beta[i][j] - 1), "64");
                model.addConstr(tau_fbdn[0][2][i][j] <= dsp_fbdn[0][i][1] - dsp_fbdn[0][i][0], "65");
                model.addConstr(tau_fbdn[0][2][i][j] >= (dsp_fbdn[0][i][1] - dsp_fbdn[0][i][0]) - (2 - beta[i][j] - beta_fbdn[j]) * dsp_max, "66");
                model.addConstr(tau_fbdn[0][2][i][j] >= 0, "67");
*/
                exp_clb += tau[0][i][j];
//                exp_clb_fbdn  += tau_fbdn[0][0][i][j] + tau_fbdn[1][0][i][j];

                exp_bram += tau[1][i][j];
//                exp_bram_fbdn += tau_fbdn[0][1][i][j];

                exp_dsp  += tau[2][i][j];
//                exp_dsp_fbdn  += tau_fbdn[0][2][i][j];
            }

            model.addConstr(clb_per_tile * (exp_clb /*- exp_clb_fbdn*/) >= clb_req_vcu128[i],"68");
            model.addConstr(wasted[i][0] == clb_per_tile * (exp_clb /*- exp_clb_fbdn*/) - clb_req_vcu128[i],"168"); //wasted clbs

//            model.addConstr(clb_fbdn_tot[i] == exp_clb_fbdn, "169");

            model.addConstr(bram_per_tile * (exp_bram /*- exp_bram_fbdn*/) >= bram_req_vcu128[i],"69");
            model.addConstr(wasted[i][1] == bram_per_tile * (exp_bram /*- exp_bram_fbdn*/) - bram_req_vcu128[i],"169"); //wasted brams

//            model.addConstr(bram_fbdn_tot[i] == exp_bram_fbdn, "169");

            model.addConstr(dsp_per_tile * (exp_dsp /*- exp_dsp_fbdn*/) >= dsp_req_vcu128[i],"70");
            model.addConstr(wasted[i][2] == dsp_per_tile * (exp_dsp /*- exp_dsp_fbdn*/) - dsp_req_vcu128[i], "170");

//            model.addConstr(dsp_fbdn_tot[i] == exp_dsp_fbdn, "169");
        }


        //Interference constraints
        /***********************************************************************
        Constraint 3.0: The semantics of Gamma, Alpha, Omega                 for(l = 0; l < 10; l++)
                    exp += z[1][i][k][l];

                model.addConstr(exp <= 6, "49000");& Psi must be fixed
        ***********************************************************************/
        for(i = 0; i < num_slots; i++) {
            GRBLinExpr exp;
            for(k = 0; k < num_slots; k++) {
                if(i == k)
                    continue;
                model.addConstr(BIG_M * gamma[i][k] >= x[k][0] + 1 - x[i][0], "63");
                model.addConstr(BIG_M * theta[i][k] >= (y[k] - y[i]), "64");
                model.addConstr(BIG_M * Gamma[i][k] >= x[i][1] - x[k][0] + 1, "65");
                model.addConstr(BIG_M * Alpha[i][k] >= x[k][1] - x[i][0] + 1, "66");
                model.addConstr(BIG_M * Omega[i][k] >= y[i] + h[i] - y[k], "67");
                model.addConstr(BIG_M * Psi[i][k]   >= y[k] + h[k] - y[i], "68");
            }
        }

        /***********************************************************************
        Constraint 3.1 Non interference between slot 'i' and 'k'
        ************************************************************************/

        for(i = 0; i < num_slots; i++) {
            for(k = 0; k < num_slots; k++) {
                if(i == k)
                    continue;

                model.addConstr(delta[0][i][k] >= gamma[i][k] + theta[i][k] +
                                Gamma[i][k] + Omega[i][k] - 3, "69");
                model.addConstr(delta[0][i][k] >= (1- gamma[i][k]) + theta[i][k] +
                                Alpha[i][k] + Omega[i][k] - 3, "70");
                model.addConstr(delta[0][i][k] >= gamma[i][k] + (1 - theta[i][k]) +
                                Gamma[i][k] + Psi[i][k] - 3, "71");
                model.addConstr(delta[0][i][k] >= (1 - gamma[i][k]) + (1 - theta[i][k]) +
                                Alpha[i][k] + Psi[i][k] - 3, "72");
                model.addConstr(delta[0][i][k] == 0, "73");
            }
        }

        /*************************************************************************
        Constraint 4.1:
        **************************************************************************/

        /*for(i = 0; i < num_slots; i++) {
            for (j = 0; j <  num_fbdn_edge; j++) {
                l = 0;
                model.addConstr(x[i][0] - forbidden_boundaries_left[j] <= -0.01 + kappa[i][j][l] * BIG_M, "edge_con");
                model.addConstr(x[i][0] - forbidden_boundaries_left[j] >= 0.01 - (1 - kappa[i][j][l]) * BIG_M, "edge_con_1");
                l++;
                model.addConstr(x[i][1] - forbidden_boundaries_right[j] <= -0.01 + kappa[i][j][l] * BIG_M, "edge_con_2");
                model.addConstr(x[i][1] - forbidden_boundaries_right[j] >= 0.01 - (1 - kappa[i][j][l]) * BIG_M, "edge_con_3");
            }
        }*/


        //Objective function parameters definition
        /*************************************************************************
        Constriant 5.0: The centroids of each slot and the distance between each
                        of them (the wirelength) is defined in these constraints.
                        The wirelength is used in the objective function
        *************************************************************************/
        GRBLinExpr obj_x, obj_y, obj_wasted_clb, obj_wasted_bram, obj_wasted_dsp;
        unsigned long wl_max = 0;

        for(i = 0; i < num_slots; i++) {
            model.addConstr(centroid[i][0] == x[i][0] + w[i] / 2, "84");
            model.addConstr(centroid[i][1] == y[i] * 10 + h[i] * 10 / 2, "86");
        }

       for(i =0; i < num_slots; i++){
            for(j = 0; j < num_slots; j++) {
                if(i >= j ) {
                    continue;
                }
                model.addConstr(dist[i][j][0] >= (centroid[i][0] - centroid[j][0]), "87");
                model.addConstr(dist[i][j][0] >= (centroid[j][0] - centroid[i][0]), "88");
                model.addConstr(dist[i][j][1] >= (centroid[i][1] - centroid[j][1]), "89");
                model.addConstr(dist[i][j][1] >= (centroid[j][1] - centroid[i][1]), "90");
            }
        }

        for(i = 0; i < num_slots; i++) {
            for(j = 0; j < num_slots; j++) {
                if(i >= j)
                    continue;
                obj_x += dist[i][j][0];
                obj_y += dist[i][j][1];
            }
        }

        for(i = 0; i < num_slots; i++) {
            obj_wasted_clb  += wasted[i][0];
            obj_wasted_bram += wasted[i][1];
            obj_wasted_dsp  += wasted[i][2];
        }


         model.setObjective((obj_x), GRB_MINIMIZE);
        //model.setObjective(obj_wasted_clb,  GRB_MINIMIZE);
        //model.setObjective(obj_wasted_bram, GRB_MINIMIZE);
         //model.setObjective(0.1 * obj_wasted_dsp,  GRB_MINIMIZE);

        //Optimize
        /****************************************************************************
        Optimize
        *****************************************************************************/
        model.set(GRB_IntParam_Threads, 8);
        model.set(GRB_DoubleParam_TimeLimit, 1800);
        model.set(GRB_DoubleParam_IntFeasTol, 1e-9);
        model.optimize();
        wasted_clb_vcu128 = 0;
        wasted_bram_vcu128 = 0;
        wasted_dsp_vcu128 = 0;
        unsigned long w_x = 0, w_y = 0;

        status = model.get(GRB_IntAttr_Status);

        if(status == GRB_OPTIMAL || status == GRB_TIME_LIMIT) {
            cout << "---------------------------------------------------------------------\
--------------------------------------------------------------------------- "<< endl;
            cout<< "slot \t" << "x_0 \t" << "x_1 \t" << "y \t" << "w \t" << "h \t" << "clb_0 \t"
                    << "clb_1 \t" << "clb \t" << "req\t" << "bram_0 \t" << "bram_1 \t" << "bram \t"
                    << "req\t" << "dsp_0 \t" << "dsp_1 \t" << "dsp\t" << "req" <<endl;
            cout << "----------------------------------------------------------------------\
------------------------------------------------------------------------" << endl;

                for(i = 0; i < num_slots; i++) {
                    cout << i << "\t" << x[i][0].get(GRB_DoubleAttr_X) <<"\t"
                        << x[i][1].get(GRB_DoubleAttr_X) << "\t" << y[i].get(GRB_DoubleAttr_X)
                        <<" \t" <<  w[i].get(GRB_DoubleAttr_X) << "\t" << h[i].get(GRB_DoubleAttr_X)

                        <<"\t" << clb[i][0].get(GRB_DoubleAttr_X) <<"\t" <<
                        clb[i][1].get(GRB_DoubleAttr_X) << "\t" << ((clb[i][1].get(GRB_DoubleAttr_X) -
                         clb[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) - clb_fbdn_tot[i].get(GRB_DoubleAttr_X)) * clb_per_tile<< "\t" << clb_req_vcu128[i]

                        <<"\t" << bram[i][0].get(GRB_DoubleAttr_X) <<"\t" <<
                        bram[i][1].get(GRB_DoubleAttr_X) << "\t" << ((bram[i][1].get(GRB_DoubleAttr_X) -
                        bram[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) - bram_fbdn_tot[i].get(GRB_DoubleAttr_X)) * bram_per_tile<< "\t" << bram_req_vcu128[i]

                        << "\t" << dsp[i][0].get(GRB_DoubleAttr_X) << "\t" <<
                        dsp[i][1].get(GRB_DoubleAttr_X) << "\t" << ((dsp[i][1].get(GRB_DoubleAttr_X) -
                                dsp[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) - dsp_fbdn_tot[i].get(GRB_DoubleAttr_X)) * dsp_per_tile << "\t" << dsp_req_vcu128[i] <<endl;

//                        cout <<endl;

                        (*to_sim->x)[i] = (int) x[i][0].get(GRB_DoubleAttr_X);
                        (*to_sim->y)[i] = (int) y[i].get(GRB_DoubleAttr_X) * 10;
                        (*to_sim->w)[i] = (int) w[i].get(GRB_DoubleAttr_X);
                        (*to_sim->h)[i] = (int) h[i].get(GRB_DoubleAttr_X) * 10;
                        (*to_sim->clb_from_solver)[i] = (int) ((clb[i][1].get(GRB_DoubleAttr_X) - clb[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) * clb_per_tile);
                        (*to_sim->bram_from_solver)[i] = (int) ((bram[i][1].get(GRB_DoubleAttr_X) - bram[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) * bram_per_tile);
                        (*to_sim->dsp_from_solver)[i] = (int) ((dsp[i][1].get(GRB_DoubleAttr_X) - dsp[i][0].get(GRB_DoubleAttr_X)) * h[i].get(GRB_DoubleAttr_X) * dsp_per_tile);
            }
            cout <<endl;

            for (i = 0; i < num_slots; i++) {
                wasted_clb_vcu128  +=  wasted[i][0].get(GRB_DoubleAttr_X);
                wasted_bram_vcu128 +=  wasted[i][1].get(GRB_DoubleAttr_X);
                wasted_dsp_vcu128  +=  wasted[i][2].get(GRB_DoubleAttr_X);

                cout << "wasted clb " << wasted[i][0].get(GRB_DoubleAttr_X) <<
                        " wasted bram " << wasted[i][1].get(GRB_DoubleAttr_X) <<
                        " wasted dsp " << wasted[i][2].get(GRB_DoubleAttr_X) <<endl;
            }

            cout <<endl;
            cout << "total wasted clb " <<wasted_clb_vcu128 <<
                    " total wasted bram " <<wasted_bram_vcu128 <<
                    " total wasted dsp " << wasted_dsp_vcu128 <<endl;

            cout <<endl;
    }

    else {

            model.set(GRB_IntParam_Threads, 8);
            model.set(GRB_DoubleParam_TimeLimit, 120);
            model.computeIIS();

        cout<< "the following constraints can not be satisfied" <<endl;
        c = model.getConstrs();

        for(i = 0; i < model.get(GRB_IntAttr_NumConstrs); i++)
            if(c[i].get(GRB_IntAttr_IISConstr) == 1)
                cout << c[i].get(GRB_StringAttr_ConstrName) << endl;
    }
}
    catch(GRBException e)
    {
        cout << "Error code =" << e.getErrorCode() << endl;
        cout<< e.getMessage() << endl;

        return 0;
    }
    catch (...)
    {
        cout <<"exception while solving milp" << endl;
        return 0;
    }
   return status;
}

int vcu128_start_optimizer(param_to_solver *param, param_from_solver *to_sim)
{
    int m = 0;
    int k = 0;
    int temp;
    unsigned long i;

    num_slots = param->num_rm_partitions;
    num_forbidden_slots = param->num_forbidden_slots;
    num_rows = param->num_rows;
    H =  param->num_clk_regs;
    W =  param->width;

    num_clk_regs = param->num_clk_regs;
    clb_per_tile = param->clb_per_tile;
    bram_per_tile = param->bram_per_tile;
    dsp_per_tile = param->dsp_per_tile;

    for(i = 0; i < num_slots; i++) {
        clb_req_vcu128[i]  = (*param->clb)[i];
        bram_req_vcu128[i] = (*param->bram)[i];
        dsp_req_vcu128[i]  = (*param->dsp)[i];
        //cout << "clb " << clb_req_vcu128[i] << " bram " << bram_req_vcu128[i] << "dsp " << dsp_req_vcu128[i] <<endl;
        cout << "clb " << clb_per_tile << " bram " << bram_per_tile << "dsp " << dsp_per_tile <<endl;
        //bram_req_pynq[i] << "dsp " << dsp_req_pynq[i] << endl;
    }

    m =0;

    cout << "FLORA starting Optimizer " << endl;
/*
    for(i = 0; i < num_forbidden_slots; i++) {
        fs_vcu128[i] = (*param->fbdn_slot)[i];
        cout <<"forbidden " << num_forbidden_slots << " " <<
               fs_vcu128[i].x << " " << fs_vcu128[i].y << " " <<
               fs_vcu128[i].h << " " << fs_vcu128[i].w <<endl;
    }
*/
    status = solve_milp_vcu128(to_sim);

return 0;
}

