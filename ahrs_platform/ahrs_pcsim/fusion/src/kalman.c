#include <stdlib.h>
#include <memory.h>
#include "types.h"
#include "kalman.h"




/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static DBL** mallocArray2D_DBL(U32 row, U32 col)
{
    DBL **pmatrix = NULL;
    U32 i = 0;
    U32 sizeofDBL = sizeof(DBL);
    U32 sizeofPtrDBL = sizeof(DBL *);

    pmatrix = (DBL **)malloc(sizeofPtrDBL * row);

    if (pmatrix == NULL)
    {
        return NULL;
    }

    for (i = 0; i < row; i++)
    {
        pmatrix[i] = (DBL *)malloc(sizeofDBL * col);

        if (pmatrix[i] == NULL)
        {
            U32 j;

            for (j = 0; j < i; j++)
            {
                free(pmatrix[j]);
            }
            free(pmatrix);
            
            return NULL;
        }
        memset(pmatrix[i], 0, sizeofDBL * col);
    }

    return pmatrix;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static U32 freeArray2D_DBL(DBL **pmatrix, U32 row, U32 col)
{
    U32 i = 0;
    PARAMETER_NOT_USED(col);

    for (i = 0; i < row; i++)
    {
        free(pmatrix[i]);
    }

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 kalmanInit(kalmanInfo_t *pkalmanInfo, U32 ustateNum)
{
    U32 sizeofDBL = sizeof(DBL);

    pkalmanInfo->uStateNum = ustateNum;
    pkalmanInfo->uUdNum = ustateNum * (ustateNum + 1) / 2;

    // malloc state array
    pkalmanInfo->pStateX = (DBL *)malloc(sizeofDBL * pkalmanInfo->uStateNum);
    if (pkalmanInfo->pStateX == NULL)
    {
        return -1;
    }
    memset(pkalmanInfo->pStateX, 0, sizeofDBL * pkalmanInfo->uStateNum);
    
    // malloc ud array
    pkalmanInfo->pUd = (DBL *)malloc(sizeofDBL * pkalmanInfo->uUdNum);
    if (pkalmanInfo->pUd == NULL)
    {
        free(pkalmanInfo->pStateX);
        return -1;
    }
    memset(pkalmanInfo->pUd, 0, sizeofDBL * pkalmanInfo->uUdNum);

    // malloc q array
    pkalmanInfo->pQd = (DBL **)mallocArray2D_DBL(pkalmanInfo->uStateNum, pkalmanInfo->uStateNum);
    if (pkalmanInfo->pQd == NULL)
    {
        free(pkalmanInfo->pStateX);
        free(pkalmanInfo->pUd);
        return -1;
    }

    // malloc phi array
    pkalmanInfo->pPhi = (DBL **)mallocArray2D_DBL(pkalmanInfo->uStateNum, pkalmanInfo->uStateNum);
    if (pkalmanInfo->pPhi == NULL)
    {
        free(pkalmanInfo->pStateX);
        free(pkalmanInfo->pUd);
        freeArray2D_DBL(pkalmanInfo->pQd, pkalmanInfo->uStateNum, pkalmanInfo->uStateNum);
        return -1;
    }

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 UDInit(DBL *pud, U32 len1, const DBL rms[], U32 len2)
{
    U32 i = 0;
    U32 j = 0;
    U32 k = 1;

    for (i = 0; i < len1; i++)
    {
        if (i == j)
        {
            if (k > len2)
            {
                return -1;
            }
            pud[i] = rms[k-1] * rms[k-1];
            k++;
            j = k * (k + 1) / 2 - 1;
        }
        else
        {
            pud[i] = 0;
        }
    }

    return 0;
}